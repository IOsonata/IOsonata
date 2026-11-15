#!/usr/bin/env python3
"""Run the shipped wizard loader without Eclipse (requires a Java 21 JDK).

Only Eclipse logging is stubbed. Descriptor parsing and MCU discovery execute
from the unmodified plugin JAR in Installer/eclipse_plugin.
"""

from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

SOURCES = {
    'CheckWizard.java': """import org.iosonata.embedcdt.templates.wizard.ui.IosManifestLoader;
public class CheckWizard {
 public static void main(String[] args) {
  var manifest = IosManifestLoader.load(args[0]);
  for (var entry : manifest.byFamily.entrySet()) {
   System.out.println(entry.getKey() + "=" + entry.getValue());
  }
  if (manifest.descriptors.size() != Integer.parseInt(args[1])) {
   throw new AssertionError("Wrong descriptor count: " + manifest.descriptors.size());
  }
  System.out.println("PASS: " + manifest.descriptors.size() + " descriptors, " + manifest.byFamily.size() + " families");
 }
}
""",
    'org/eclipse/core/runtime/ILog.java': """package org.eclipse.core.runtime;
public interface ILog { void log(IStatus status); }
""",
    'org/eclipse/core/runtime/IStatus.java': """package org.eclipse.core.runtime;
public interface IStatus { }
""",
    'org/eclipse/core/runtime/Platform.java': """package org.eclipse.core.runtime;
public class Platform {
 public static ILog getLog(Class<?> owner) { return status -> {}; }
}
""",
    'org/eclipse/core/runtime/Status.java': """package org.eclipse.core.runtime;
public class Status implements IStatus {
 public Status(int severity, String plugin, String message) { }
}
""",
}


def main():
    jars = list((ROOT / "Installer/eclipse_plugin").glob(
        "org.iosonata.embedcdt.templates.wizard_*.jar"))
    if not jars:
        raise SystemExit("No bundled wizard plugin found")
    jar = max(jars, key=lambda p: tuple(int(n) for n in re.findall(
        r"\d+", p.stem)))
    count = sum(1 for arch in ("ARM", "RISCV")
                for path in (ROOT / arch).glob("**/lib/ioc/wizard.json"))
    with tempfile.TemporaryDirectory(prefix="iosonata-wizard-") as directory:
        work = Path(directory)
        sources = []
        for relative, content in SOURCES.items():
            path = work / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
            sources.append(str(path))
        classes = work / "classes"
        subprocess.run(["java", "-m", "jdk.compiler/com.sun.tools.javac.Main",
                        "-cp", str(jar), "-d", str(classes), *sources], check=True)
        subprocess.run(["java", "-cp", os.pathsep.join((str(classes), str(jar))),
                        "CheckWizard", str(ROOT), str(count)], check=True)


if __name__ == "__main__":
    main()
