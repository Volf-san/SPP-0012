'use strict'

const version = Object.freeze(new Version(2, 0, 2))
console.log('Loading CogUtils')
console.log('CogUtils Version: ' + version.toString())

var fs = require('fs')
function Version (major, minor, patch, suffix) {
  if (arguments.length < 3) {
    throw new Error('To few arguments!')
  }

  if (arguments.length < 4) {
    suffix = ''
  }

  return {
    get Major () { return major },
    get Minor () { return minor },
    get Patch () { return patch },
    toString: function () {
      let version = major.toString() + '.' + minor.toString() + '.' + patch
      if ((suffix) && (suffix.length > 0)) {
        version = version + '-'
      }
      return version + suffix
    }
  }
};

var logToConsole = 1
const log = (logToConsole == 1 ? _log : function () { })

function _log (msg) {
  // if (msg != null) {
  if (typeof msg === 'object') {
    console.log('----> ' + msg.constructor.name)
    console.log(JSON.stringify(msg))
    console.log('<---- ' + msg.constructor.name)
  } else {
    console.log(msg)
  }
  
  // }
  // else {
  //	console.log("Message is <NULL>");
  // }
}

function loadFile (filename) {
  try {
    var data = {}
    if (fs.existsSync(filename)) {
      data = JSON.parse(fs.readFileSync(filename))
      console.log('File <' + filename + '> loaded!')
    } else {
      console.log('File <' + filename + "> doesn't exist!")
      data = {}
    }
  } catch (e) {
    console.log('Exception during loading file <' + filename + '>!\r\n' + e)
  } finally {
    return data
  }
}

function saveFile (filename, data) {
  try {
    fs.writeFileSync(filename, JSON.stringify(data))
  } catch (e) {
    console.log('Exception during saving file <' + filename + '>\r\n' + e)
  }
}

function deleteFile (filename) {
  var ret = false
  var fileToDelete=''
  try {
    //fs.unlinkSync(filename)
    let files = fs.readdirSync('.')
    var regex = new RegExp(filename)
    for(var i = 0, len = files.length; i < len; i++) {
      var match = files[i].match(regex)
      if(match !== null){
        fileToDelete = files[i]
             fs.unlinkSync(match[0]);
      }
    }

    ret = true
  } catch (e) {
    console.log('Exception during deleting file <' + fileToDelete + '>\r\n' + e)
    ret = false
  }

  return ret
}
function assert (condition, message) {
  if (!condition) {
    message = message || 'Assertion failed'
    if (typeof Error !== 'undefined') {
      throw new Error(message)
    }
    throw message // Fallback
  }
};

module.exports.loadFile = loadFile
module.exports.saveFile = saveFile
module.exports.deleteFile = deleteFile
module.exports.assert = assert
module.exports.log = log
module.exports.Version = Version
module.exports.version = version

console.log('Loading CogUtils done')
