/**
 * Updater used to update a Sphinx configuration file (i.e. `conf.py`).
 */

/**
 * Reads the current version from the Sphinx configuration file.
 */
module.exports.readVersion = function (contents) {
  // console.debug(`file contents:\n\n${contents}`);
  const version = contents.match(/(?<=version = ")\d+\.\d+\.\d+(?=")/g)[0];
  // console.debug("found version:", version);
  return version;
};

/**
 * Writes the new version to the Sphinx configuration file.
 */
module.exports.writeVersion = function (contents, version) {
  // console.debug(`file contents:\n\n${contents}`);
  return contents
    .replace(/(?<=version = ")\d+\.\d+\.\d+(?=")/g, () => {
      // console.debug("replace version with", version);
      return version;
    })
    .replace(/(?<=release = ")\d+\.\d+\.\d+(?=")/g, () => {
      // console.debug("replace release version with", version);
      return version;
    });
};
