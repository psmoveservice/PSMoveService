#!/bin/bash -x -e
#

{
PLUGIN_SRC_DIR=$(pwd)
BUILD_DIR=$PLUGIN_SRC_DIR/../../build/src
PLUGIN_BUILD_DIR=$BUILD_DIR/openvr_plugin/Debug
CLIENT_BUILD_DIR=$BUILD_DIR/psmoveclient/Debug
INSTALL_DIR=drivers/psmove/bin/osx32

cd ~/Library/Application\ Support/Steam/steamapps/common/SteamVR
mkdir -p $INSTALL_DIR
cp $PLUGIN_BUILD_DIR/libdriver_psmoveservice.dylib $INSTALL_DIR/
cp $CLIENT_BUILD_DIR/libPSMoveClient.dylib $INSTALL_DIR/
bin/osx32/vrpathreg adddriver $INSTALL_DIR
cp -R $PLUGIN_SRC_DIR/resources ~/Library/Application\ Support/Steam/steamapps/common/SteamVR/drivers/psmove/
cd $PLUGIN_SRC_DIR
} &> /dev/null
echo "Don't forget to edit your config/steamvr.vrsettings to enable 'activateMultipleDrivers' and possibly set 'requireHmd':false";
