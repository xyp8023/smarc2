# Scripts
A pile of bash or python scripts to make life easier.

List them here with a simple explanation so people know what they are running ;)

#### get-submodules.sh
Usage:
```bash
cd colcon_ws/smarc2
./scripts/get-submodules.sh <foldername>
```
where `foldername` is a folder (or its first few characters, like `ext` for `external`).

Example: `./scipts/get-submodules.sh ext` will update all submodules in the folder `smarc2/external`. `./scripts/get-submodules.sh sim` will do the same for `smarc2/simulation`.