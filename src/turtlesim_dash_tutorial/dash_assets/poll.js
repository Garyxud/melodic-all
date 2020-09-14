// Function to fire an event on a DOM object
function eventFire(el, etype){
  if (el.fireEvent) {
    el.fireEvent('on' + etype);
  } else {
    var evObj = document.createEvent('Events');
    evObj.initEvent(etype, true, false);
    el.dispatchEvent(evObj);
  }
}

// Function to check if 2 objects are different
function objAreDiff(obj1, obj2) {
    // Check that the lengths are the same at the very least
    if (Object.keys(obj1).length !== Object.keys(obj2).length) {
        return true;
    }

    // Iterate through the keys and check
    for (let key in obj1) {
        if (obj1[key] !== obj2[key]) {
            return true;
        }
    }

    // Nothing is different
    return false;
}

// After a set timeout, make sure to check for an update to the enabled status.
// Trigger a series of updates only IFF there has been an update to the enabled
// status
var ros_status = {
    server_status: null
};

function check_status() {
    // Send an AJAX request to see if status has updated on the remote server
    fetch('/ros_api/status')
        .then(function(data) {
            return data.json();
        })
        .then(function(data) {
            if (data.server_status !== ros_status.server_status) {
                eventFire(document.getElementById('refresh-status'), 'click');
            }
            ros_status = data;
            return null;
        })
        .catch(console.error);
}

// Poll for a status update every second
var poll_interval = setInterval(check_status, 1000);
