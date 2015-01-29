Installing Voice Recognition Packages

You basically need 2 packages in order to do voice recognition.

1. Install Pocketsphinx: 
  
  Either: http://cmusphinx.sourceforge.net/wiki/tutorialpocketsphinx#installation and http://cmusphinx.sourceforge.net/wiki/download
  
  Or: sudo apt-get install ros-<version>-pocketsphinx <- This method not guaranteed to contain lmtool
  
2. Install Sphinxbase:
  
  Follow the same link as above: http://cmusphinx.sourceforge.net/wiki/download
  
lmtool is a tool to take phrases that you want the robot to recognise and outputs a file of robot-readable instructions. It comes packaged with PocketSphinx. Check out the README at http://sourceforge.net/p/cmusphinx/code/HEAD/tree/trunk/logios/

This is a work in progress and hopefully this folder will later contain mirrors of these packages and standalone set up instructions. 
