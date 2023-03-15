var express = require('express');
var router = express.Router();
var degree = -20;

/* get current time */
function formattedDateTime(date) {
  const y = date.getFullYear();
  const m = ('0' + (date.getMonth() + 1)).slice(-2);
  const d = ('0' + date.getDate()).slice(-2);
  // const h = ('0' + date.getHours()).slice(-2);
  // const mi = ('0' + date.getMinutes()).slice(-2);
  // const s = ('0' + date.getSeconds()).slice(-2);
  const ms = date.getHours() * 60 * 60 * 1000
           + date.getMinutes() * 60 * 1000
           + date.getSeconds() *1000
           + date.getMilliseconds();
  return y + m + d + "," + String(ms);
}

/* GET home page. */
router.get('/', function(req, res, next) {
  res.writeHead(200, {'Content-Type': 'text/plain'});
  res.write(mode + ",");
  res.write(degree + ",");
  res.write(current_time);
  res.end();
});

router.post('/', function(req, res, next) {
  // const date = new Date();
  // need offset of Time Zone
  const date = new Date(Date.now() + ((new Date().getTimezoneOffset() + (9 * 60)) * 60 * 1000));
  current_time = formattedDateTime(date);
  degree = req.body.degree;
  mode = req.body.mode;
  console.log("POSTED MODE: ", mode);
  console.log("POSTED DEGREE: ", degree);
  console.log("CURRENT TIME: ", current_time);
});

module.exports = router;
