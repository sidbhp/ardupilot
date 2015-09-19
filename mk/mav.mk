git remote set-url mavlink https://github.com/3drobotics/mavlink_c_headers
git branch -f mavlink mavlink/master
git checkout -f mavlink
git subtree add --prefix==$(SKETCHBOOK)/library/GCS_MAVLINK/mavlink mavlink