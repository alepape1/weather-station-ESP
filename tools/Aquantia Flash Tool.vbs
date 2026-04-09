CreateObject("WScript.Shell").Run "pythonw """ & CreateObject("Scripting.FileSystemObject").GetParentFolderName(WScript.ScriptFullName) & "\flasher_gui.py""", 0, False
