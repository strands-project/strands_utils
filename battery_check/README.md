Battery check
=============

This script checks for the battery level and if it drops below certain threshold, it sends a notification over an email.
Should the battery level fall below 30%, the script will attempt to shut down the PC.

The email contains a link to a detailed recovery procedure with pictures, so the recovery can be performed by inexperienced users ( maybe even professors ).

Prerequisities:

Mailx email client: http://en.wikipedia.org/wiki/Mailx must be configured.

ROS must be running so that battery status can be read.

Cron must be on.

To install:

1) You need to allow a nonroot user to shutdown the PC.

run 'sudo visudo' and add the following line (replace user and hostname with the correct values)

user hostname =NOPASSWD: /usr/bin/shutdown -h now,/usr/bin/halt,/usr/bin/poweroff,/usr/bin/reboot

2) Edit and copy the .batterycheck to the home directory.

To set up where the emails are send, change the adresses in the first two lines of the .batterycheck file. 

Copy the file to the home directory:  cp .batterycheck ~/

3) Tell cron to run the script every 30 minutes.

run crontab -e
and add

*/30 * * * * bash path_to/batterycheck.sh


4) Make the script executeable and run it.

chmod +x batterycheck
run ./batterycheck.sh test

A warning message concerning the battery status should be displayed and you should get an email.


Issues:

ROS must be running so that battery status can be read - this might be substituted by MIRA driver if we'll experience issues with ROS.

