import os
import qi


app = qi.Application()
qicore = qi.module('qicore')

log = qi.Logger("qicore.file.example.AliceServices.ImageStore")

class ImageStore:
  def __init__(self):
    self.file_registry = dict()
    self.counter = 0

  def __del__(self):
    for name, path in self.file_registry.iteritems():
       os.remove(path)

  # Store a copy of the image file and associate it with the provided name.
  def store_image(self, image_file, name):
    # Note that ideally this would be implemented in an asynchronous way,
    # but for simplicity we will do it synchronously.

    log.info("Storing '{0}' file ...".format(name))

    # First, make a local copy in a temporary files directory:
    temp_file_path = self._generate_temporary_file_path()

    # This call will block until the end because it returns a FutureSync.
    qicore.copyToLocal(image_file, temp_file_path)

    # We now have a local copy of the remote file,
    # so we don't need the remote access anymore.
    del image_file

    # Now we can work with the local file.
    self._store_file_in_database(name, temp_file_path)

  # Provide access to an image file associated with the provided name.
  def get_image(self, name):
    log.info("Getting '{0}' file ...".format(name))
    file_location = self._find_file_location(name)

    if file_location:
      log.info("'{0}' found at {1}".format(name, file_location))

      # Now we can open it and provide it to the user for reading.
      return qicore.openLocalFile(file_location)
    else:
      return None # File not found!

  def _generate_temporary_file_path(self):
    self.counter = self.counter + 1
    return "./temp_image_file_{0}.data".format(self.counter)

  def _store_file_in_database(self, name, path):
    #...fake it
    self.file_registry[name] = path
    log.info("'{0}' stored at {1}".format(name, path))

  def _find_file_location(self, name):
    return self.file_registry[name]



app.start()

session = app.session
session.registerService("ImageStore", ImageStore())

app.run()

