/*
This is a Frida (https://frida.re) script used to intercept TLS keys used by the
Dashboard to communicate with the SeaTrac server.

At the time of writing, the Dashboard uses the libssl-1_1-x64.dll library for
its TLS connection. This version of libssl does not support the SSLKEYLOGFILE
environment variable.

Keys will be written to C:\dashboard\sslkeys.log and can be loaded in Wireshark
to decrypt the traffic. See README.md for more details.
*/

"use strict";

function hookLibssl(module) {
  console.log("[*] Trying to hook", module.name);

  // We are going to hook all new SSL_CTX objects and attach a keylog
  // callback to them.
  //
  // First, find all the relevant exports in the module.
  const SSL_CTX_new = module.findExportByName("SSL_CTX_new");
  const SSL_CTX_new_ex = module.findExportByName("SSL_CTX_new_ex");
  if (!SSL_CTX_new) {
    // SSL_CTX_new_ex() is optional
    console.log("[-] Cannot locate SSL_CTX_new()");
    return;
  }

  let SSL_CTX_set_keylog_callback = module.findExportByName(
    "SSL_CTX_set_keylog_callback",
  );
  if (!SSL_CTX_set_keylog_callback) {
    console.log("[-] Cannot locate SSL_CTX_set_keylog_callback() functions");
    return;
  }

  // Convert the SSL_CTX_set_keylog_callback pointer to a callable
  SSL_CTX_set_keylog_callback = new NativeFunction(
    SSL_CTX_set_keylog_callback,
    "void",
    ["pointer", "pointer"],
  );

  // Generate a callback that will write the key material to our log file. The
  // caller already formats the log entry.
  const callback = new NativeCallback(
    (ctx, line) => {
      const file = new File("C:\\dashboard\\sslkeys.log", "a");
      file.write(line.readUtf8String() + "\n");
      file.close();
      console.log("[+] Logged key material from SSL context", ctx);
    },
    "void",
    ["pointer", "pointer"],
  );

  function wrap_SSL_CTX_new(fn) {
    Interceptor.attach(fn, {
      onLeave: function (ctx) {
        if (ctx.isNull()) return;
        SSL_CTX_set_keylog_callback(ctx, callback);
        console.log("[+] Added keylog callback on SSL context", ctx);
      },
    });
  }

  if (SSL_CTX_new) wrap_SSL_CTX_new(SSL_CTX_new);
  if (SSL_CTX_new_ex) wrap_SSL_CTX_new(SSL_CTX_new_ex);
}

// The libssl DLL is not loaded already at the time our script loads, so we
// monitor modules as they're loaded and hook libssl on the fly.
function hookOnLoad(name, isWide) {
  const fn = Module.findExportByName("kernel32.dll", name);
  Interceptor.attach(fn, {
    onEnter: function (args) {
      this.moduleName = isWide
        ? args[0].readUtf16String()
        : args[0].readUtf8String();
    },
    onLeave: function (retval) {
      if (retval.isNull()) return;
      if ((this.moduleName || "").match(/(^|[\\/])libssl.*\.dll$/i)) {
        console.log("[+] Found load of SSL module", this.moduleName);
        hookLibssl(Process.getModuleByAddress(retval));
      }
    },
  });
}

hookOnLoad("LoadLibraryA", false);
hookOnLoad("LoadLibraryW", true);
