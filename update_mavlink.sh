git remote set-url mavlink https://github.com/3drobotics/mavlink_c_headers
git subtree add --prefix library/GCS_MAVLINK/mavlink mavlink master --squash
git subtree pull --prefix library/GCS_MAVLINK/mavlink mavlink master --squash