//Eyes Controller
class EyeController {
  constructor(elements = {}, eyeSize = '33.33vmin') {
    this._eyeSize = eyeSize;
    this._blinkTimeoutID = null;

    this.setElements(elements);
  }

  get leftEye() { return this._leftEye; }
  get rightEye() { return this._rightEye; }

  setElements({
    leftEye,
    rightEye,
    upperLeftEyelid,
    upperRightEyelid,
    lowerLeftEyelid,
    lowerRightEyelid,
  } = {}) {
    this._leftEye = leftEye;
    this._rightEye = rightEye;
    this._upperLeftEyelid = upperLeftEyelid;
    this._upperRightEyelid = upperRightEyelid;
    this._lowerLeftEyelid = lowerLeftEyelid;
    this._lowerRightEyelid = lowerRightEyelid;
    return this;
  }

  _createKeyframes ({
    tgtTranYVal = 0,
    tgtRotVal = 0,
    enteredOffset = 1/3,
    exitingOffset = 2/3,
  } = {}) {
    return [
      {transform: `translateY(0px) rotate(0deg)`, offset: 0.0},
      {transform: `translateY(${tgtTranYVal}) rotate(${tgtRotVal})`, offset: enteredOffset},
      {transform: `translateY(${tgtTranYVal}) rotate(${tgtRotVal})`, offset: exitingOffset},
      {transform: `translateY(0px) rotate(0deg)`, offset: 1.0},
    ];
  }

  express({
    type = '',
    // level = 3,  // 1: min, 5: max
    duration = 1000,
    enterDuration = 75,
    exitDuration = 75,
  }) {
    if (!this._leftEye) {  // assumes all elements are always set together
      console.warn('Eye elements are not set; return;');
      return;
    }

    const options = {
      duration: duration,
    };

    switch(type) {
      case 'happy':
        return {
          lowerLeftEyelid: this._lowerLeftEyelid.animate(this._createKeyframes({
            tgtTranYVal: `calc(${this._eyeSize} * -2 / 3)`,
            tgtRotVal: `30deg`,
            enteredOffset: enterDuration / duration,
            exitingOffset: 1 - (exitDuration / duration),
          }), options),
          lowerRightEyelid: this._lowerRightEyelid.animate(this._createKeyframes({
            tgtTranYVal: `calc(${this._eyeSize} * -2 / 3)`,
            tgtRotVal: `-30deg`,
            enteredOffset: enterDuration / duration,
            exitingOffset: 1 - (exitDuration / duration),
          }), options),
        };

      case 'sad':
        return {
          upperLeftEyelid: this._upperLeftEyelid.animate(this._createKeyframes({
            tgtTranYVal: `calc(${this._eyeSize} * 1 / 3)`,
            tgtRotVal: `-20deg`,
            enteredOffset: enterDuration / duration,
            exitingOffset: 1 - (exitDuration / duration),
          }), options),
          upperRightEyelid: this._upperRightEyelid.animate(this._createKeyframes({
            tgtTranYVal: `calc(${this._eyeSize} * 1 / 3)`,
            tgtRotVal: `20deg`,
            enteredOffset: enterDuration / duration,
            exitingOffset: 1 - (exitDuration / duration),
          }), options),
        };

      case 'angry':
        return {
          upperLeftEyelid: this._upperLeftEyelid.animate(this._createKeyframes({
            tgtTranYVal: `calc(${this._eyeSize} * 1 / 4)`,
            tgtRotVal: `30deg`,
            enteredOffset: enterDuration / duration,
            exitingOffset: 1 - (exitDuration / duration),
          }), options),
          upperRightEyelid: this._upperRightEyelid.animate(this._createKeyframes({
            tgtTranYVal: `calc(${this._eyeSize} * 1 / 4)`,
            tgtRotVal: `-30deg`,
            enteredOffset: enterDuration / duration,
            exitingOffset: 1 - (exitDuration / duration),
          }), options),
        };

      case 'focused':
        return {
          upperLeftEyelid: this._upperLeftEyelid.animate(this._createKeyframes({
            tgtTranYVal: `calc(${this._eyeSize} * 1 / 3)`,
            enteredOffset: enterDuration / duration,
            exitingOffset: 1 - (exitDuration / duration),
          }), options),
          upperRightEyelid: this._upperRightEyelid.animate(this._createKeyframes({
            tgtTranYVal: `calc(${this._eyeSize} * 1 / 3)`,
            enteredOffset: enterDuration / duration,
            exitingOffset: 1 - (exitDuration / duration),
          }), options),
          lowerLeftEyelid: this._lowerLeftEyelid.animate(this._createKeyframes({
            tgtTranYVal: `calc(${this._eyeSize} * -1 / 3)`,
            enteredOffset: enterDuration / duration,
            exitingOffset: 1 - (exitDuration / duration),
          }), options),
          lowerRightEyelid: this._lowerRightEyelid.animate(this._createKeyframes({
            tgtTranYVal: `calc(${this._eyeSize} * -1 / 3)`,
            enteredOffset: enterDuration / duration,
            exitingOffset: 1 - (exitDuration / duration),
          }), options),
        }

      case 'confused':
        return {
          upperRightEyelid: this._upperRightEyelid.animate(this._createKeyframes({
            tgtTranYVal: `calc(${this._eyeSize} * 1 / 3)`,
            tgtRotVal: `-10deg`,
            enteredOffset: enterDuration / duration,
            exitingOffset: 1 - (exitDuration / duration),
          }), options),
        }

      default:
        console.warn(`Invalid input type=${type}`);
    }
  }

  blink({
    duration = 150,  // in ms
  } = {}) {
    if (!this._leftEye) {  // assumes all elements are always set together
      console.warn('Eye elements are not set; return;');
      return;
    }

    [this._leftEye, this._rightEye].map((eye) => {
      eye.animate([
        {transform: 'rotateX(0deg)'},
        {transform: 'rotateX(90deg)'},
        {transform: 'rotateX(0deg)'},
      ], {
        duration,
        iterations: 1,
      });
    });
  }

  startBlinking({
    maxInterval = 5000
  } = {}) {
    if (this._blinkTimeoutID) {
      console.warn(`Already blinking with timeoutID=${this._blinkTimeoutID}; return;`);
      return;
    }
    const blinkRandomly = (timeout) => {
      this._blinkTimeoutID = setTimeout(() => {
        this.blink();
        blinkRandomly(Math.random() * maxInterval);
      }, timeout);
    }
    blinkRandomly(Math.random() * maxInterval);
  }

  stopBlinking() {
    clearTimeout(this._blinkTimeoutID);
    this._blinkTimeoutID = null;
  }

  setEyePosition(eyeElem, x, y, isRight = false) {
    if (!eyeElem) {  // assumes all elements are always set together
      console.warn('Invalid inputs ', eyeElem, x, y, '; retuning');
      return;
    }

    if (!!x) {
      if (!isRight) {
        eyeElem.style.left = `calc(${this._eyeSize} / 3 * 2 * ${x})`;
      } else {
        eyeElem.style.right = `calc(${this._eyeSize} / 3 * 2 * ${1-x})`;
      }
    }
    if (!!y) {
      eyeElem.style.bottom = `calc(${this._eyeSize} / 3 * 2 * ${1-y})`;
    }
  }
}

const eyes = new EyeController({
  leftEye: document.querySelector('.left.eye'),
  rightEye: document.querySelector('.right.eye'),
  upperLeftEyelid: document.querySelector('.left .eyelid.upper'),
  upperRightEyelid: document.querySelector('.right .eyelid.upper'),
  lowerLeftEyelid: document.querySelector('.left .eyelid.lower'),
  lowerRightEyelid: document.querySelector('.right .eyelid.lower'),
});
///////////////////////////////////////////////////////////////////////

//ROS
var ros = new ROSLIB.Ros();
// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('connected').style.display = 'none';
  document.getElementById('closed').style.display = 'none';
  document.getElementById('error').style.display = 'inline';
  console.log(error);
});

// Find out exactly when we made a connection.
ros.on('connection', function() {
  console.log('Connection made!');
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('error').style.display = 'none';
  document.getElementById('closed').style.display = 'none';
  document.getElementById('connected').style.display = 'inline';
});

ros.on('close', function() {
  console.log('Connection closed.');
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('connected').style.display = 'none';
  document.getElementById('closed').style.display = 'inline';
});

ros.connect('ws://localhost:9090');

var face_expression = new ROSLIB.Topic({
  ros : ros,
  name : '/face_expression',
  messageType : 'std_msgs/String'
});

face_expression.subscribe(function(message) {
  console.log('Received message on ' + face_expression.name + ': ' + message.data);
  expression=message.data;
  switch(expression){
    case 'startBlinking':
      eyes.startBlinking();
      break;
    
    case 'stopBlinking':
      eyes.stopBlinking();
      break;
    
    case 'blink':
      eyes.blink();
      break;

    case 'happy':
      eyes.express({type: 'happy'});
      break;
    
    case 'sad':
      eyes.express({type: 'sad'});
      break;
    
    case 'angry':
      eyes.express({type: 'angry'});
      break;
    
    case 'focused':
      eyes.express({type: 'focused'});
      break;
    
    case 'confused':
      eyes.express({type: 'confused'});
      break;
  }
});