cd src/tomato_gui

CY='\033[0;36m'
NC='\033[0m'

printf "${CY}Creating directories${NC}\n"

mkdir -p build
mkdir -p releaseBuild

cd build

printf "${CY}Creating release build${NC}\n"

~/Compilations/cmake-3.30.3-linux-x86_64/bin/cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../releaseBuild/gui ..
~/Compilations/cmake-3.30.3-linux-x86_64/bin/cmake --build .
~/Compilations/cmake-3.30.3-linux-x86_64/bin/cmake --install .

cd ../releaseBuild

mkdir -p gui/scripts/
mkdir -p gui/VisionConfig/

cp ~/Tomatoes/src/tomato_gui/scripts/tuck_arm.py ./gui/scripts
cp ~/Tomatoes/src/tomato_gui/VisionConfig/config.toml ./gui/VisionConfig

printf "${CY} Copyed necessary files ${NC}\n"

printf "${CY}Creating ZIP{NC}\n"

zip -r gui.zip gui/

read -p "Deploy to GitHub?" -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then
    printf "${CY}Getting latest release tag${NC}\n"

    releaseList=$(gh release list --limit 1)
    [[ $releaseList =~ v[0-9]* ]]
    previousVersion="${BASH_REMATCH:1}"

    printf "${CY}Old release tag was: $previousVersion ${NC}\n"

    newVersion=v$(($previousVersion+1))

    printf "${CY}Uploading new release${NC}\n"
    gh release create $newVersion --title rolling --notes "See Assets for zip. You will find the app inside /bin" --target master ./gui.zip

fi
