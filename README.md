# Saipranav-Vojjala

UBUNTU INSTALLATION ERRORS AND CORRECTIONS
1) Grub Screen shows error message "Failed to open \efi\boot\mmx64.efi - NOT FOUND"
  * Restart System, and boot to windows.
  * Open pen drive folder and rename grubx64.efi to mmx64.efi.  
2) Partitions created weren't visible on the Ubuntu Installer.
  * Open BIOS and change SATA from Intel RAID to AHCI
  * Boot to ubuntu live. 
  * After ubuntu installed, restart system, choose to boot windows.
  * Reinstall Windows.
