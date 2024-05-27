#!/bin/bash

PACKAGE_NAME="OnStep_X2.pkg"
BUNDLE_NAME="org.rti-zone.OnStepX2"

if [ ! -z "$app_id_signature" ]; then
    codesign -f -s "$app_id_signature" --verbose ../build/Release/libOnStep.dylib
fi

mkdir -p ROOT/tmp/OnStep_X2/
cp "../OnStep.ui" ROOT/tmp/OnStep_X2/
cp "../OnStep.png" ROOT/tmp/OnStep_X2/
cp "../mountlist OnStep.txt" ROOT/tmp/OnStep_X2/
cp "../build/Release/libOnStep.dylib" ROOT/tmp/OnStep_X2/

if [ ! -z "$installer_signature" ]; then
	# signed package using env variable installer_signature
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --sign "$installer_signature" --scripts Scripts --version 1.0 $PACKAGE_NAME
	pkgutil --check-signature ./${PACKAGE_NAME}
else
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --scripts Scripts --version 1.0 $PACKAGE_NAME
fi

rm -rf ROOT
