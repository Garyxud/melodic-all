
import sys
import qi
import qi.path
import time


app = qi.Application()
qicore = qi.module('qicore')

log = qi.Logger("qicore.file.example")

def print_tranfer_progress(progress_value):
  log.info(">>>> File Transfer Progress = {0}%".format(progress_value * 100))

def work_on_image_file(image_file_path):
  log.info("Working on image file at {0} ...".format(image_file_path))
  # we fake working on it...
  time.sleep(1)
  log.info("Working on image file at {0} - DONE".format(image_file_path))


def store_image(image_store, name, image_file_path):

  log.info("Storing image {1} at {0} into the ImageStore...".format(image_file_path, name))

  # First open the file with read-only shareable access.
  file = qicore.openLocalFile(image_file_path)

  # Now we can share reading access to this file.
  image_store.store_image(file, name)

  log.info("Storing image {1} at {0} into the ImageStore - DONE".format(image_file_path, name))


def process_image(image_store, image_file_name, image_file_path):

  # We acquire read-only access to the file and retrieve it locally.
  file = image_store.get_image(image_file_name)
  qicore.copyToLocal(file, image_file_path)

  # We don't need the remote access anymore.
  del file

  # Now work on the file located at `fileLocation`.
  work_on_image_file(image_file_path)


def process_image_with_progress(image_store, image_file_name, image_file_path):

  # We acquire read-only access to the file.
  file = image_store.get_image(image_file_name)

  # We prepare the operation without launching it yet:
  file_operation = qicore.FileCopyToLocal(file, image_file_path)

  # We want to see the progress so we plug a logging function.
  file_operation.notifier().progress.connect(print_tranfer_progress)

  # Launch the copy and wait for it to end before continuing.
  file_operation.start() # In real projects, prefer file_operation.start(_async=True).then(nextFunction) instead.

  # We don't need the remote access anymore.
  del file
  del file_operation

  # Now work on the file located at `fileLocation`.
  work_on_image_file(image_file_path)


def do_some_work(client_session, image_path, image_name):

  image_store = client_session.service('ImageStore')
  assert image_store
  store_image(image_store, image_name, image_path)
  process_image(image_store, image_name, "./tempfile")
  process_image_with_progress(image_store, image_name, "./tempfile")



path_to_file = sys.argv[1]
assert path_to_file

file_id = sys.argv[2]
assert file_id

app.start()

do_some_work(app.session, path_to_file, file_id)



