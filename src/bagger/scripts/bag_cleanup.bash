#!/usr/bin/env bash
set -o errexit

# Helper function to reindex a passed rosbag file.  The old unindexed file is then deleted
reindex_remove_orig() {
  echo "Reindexing $1"
  BAG_NAME_NO_EXTENSION=$(echo "$1" | sed -e 's/\.bag.active//g')
  BAG_NAME_REINDEXED="$BAG_NAME_NO_EXTENSION.bag.active"
  OLD_UNINDEXED_BAG_NAME="$BAG_NAME_NO_EXTENSION.bag.orig.active"
  # reindex the unindexed file
  rosbag reindex "$1"
  # remove the old unindexed file
  rm "$OLD_UNINDEXED_BAG_NAME"
  # rename the reindexed file to the original bag name
  mv "$BAG_NAME_REINDEXED" "$BAG_NAME_NO_EXTENSION.bag"
}

# Helper function for determining if a passed rosbag file is compressed, then
# uncompressing it if so.The old compressed file is then deleted
decompress_remove_orig() {
  ROSBAG_INFO_OUT=$(rosbag info "$1") 
  ROSBAG_INFO_OUT_NO_SPACES=$(echo "$ROSBAG_INFO_OUT" | sed -e 's/ //g')
  if [[ $ROSBAG_INFO_OUT_NO_SPACES = *"compression:none"* ]]; then
    echo "$1 is uncompressed"
  else
    echo "Decompressing $1"
    BAG_NAME_NO_EXTENSION=$(echo "$1" | sed -e 's/\.bag//g')
    OLD_COMPRESSED_BAG_NAME="$BAG_NAME_NO_EXTENSION.orig.bag"
    rosbag decompress "$1"
    # remove the old compressed file
    rm "$OLD_COMPRESSED_BAG_NAME"
  fi
}

# Go through all files.
# if active, reindex, then remove the orig file and rename the reindexed file
# check if the reindexed and renamed file is compressed and if so uncompress it
main() {
  for FILE_NAME in "$1"/*
  do
    echo "$FILE_NAME"
    if [[ "$FILE_NAME" = *".bag"* ]]; then
      REINDEXED_FILE_NAME="$FILE_NAME" 
      if [[ "$FILE_NAME" = *"bag.active" ]]; then
        reindex_remove_orig "$FILE_NAME"
        REINDEXED_FILE_NAME=$(echo "$FILE_NAME" | sed -e 's/\.active//g')
      fi
      if [[ $2 = false ]]; then
        decompress_remove_orig "$REINDEXED_FILE_NAME"
      fi
    else
      echo "Not touching $FILE_NAME - not a rosbag"
    fi
  done
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ] || (($# == 0)) ; then
cat <<EOF
  This script will iterate through all of the rosbags contained in a passed
  directory, first reindexing any that haven't been closed properly, then
  optionally decompressing any that are compressed.  After the decompression
  and reindexing steps, the names will be <name>.bag, and free from any .active
  tyrany.  Be advised that the old .active and compressed bags will be deleted
  automatically after this script has created a new indexed and decompressed
  version of the bag.
  
  Usage: bag_cleanup.bash [options] -d <bag_directory>
  -h | --help | Summon this help message
  -n | If passed, bags will not be decompressed, only reindexed
  -d | The directory containing bags for reindex and optional decompression
EOF
  exit 0
fi

NO_DECOMPRESS=false

while getopts ":d:n" opt; do
  case $opt in
    d)
      DIRNAME=$OPTARG
      ;;
    n)
      NO_DECOMPRESS=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      ;;
  esac
done

main "$DIRNAME" $NO_DECOMPRESS