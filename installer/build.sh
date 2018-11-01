#!/bin/bash

mkdir -p ROOT/tmp/iOptronV3_X2/
cp "../iOptronV3.ui" ROOT/tmp/iOptronV3_X2/
cp "../mountlist iOptronV3.txt" ROOT/tmp/iOptronV3_X2/
cp "../build/Release/libiOptronV3.dylib" ROOT/tmp/iOptronV3_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.iOptronV3_X2 --sign "$installer_signature" --scripts Scripts --version 1.0 iOptronV3_X2.pkg
pkgutil --check-signature ./iOptronV3_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.iOptronV3_X2 --scripts Scripts --version 1.0 iOptronV3_X2.pkg
fi

rm -rf ROOT
