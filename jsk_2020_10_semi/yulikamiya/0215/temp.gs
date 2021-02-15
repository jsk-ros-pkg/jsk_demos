function doPost(e) {
  const green = [120, 255, 120]
  const red = [255, 120, 120]
  const yellow = [255, 255, 120]
  const ss = SpreadsheetApp.getActiveSpreadsheet();
  const sheet = SpreadsheetApp.getActiveSheet();
  var jsonString = e.postData.getDataAsString();
  var data = JSON.parse(jsonString);
  var name = data.name;
  var temp = data.temp;
  var time = data.time+32400; //時差を考慮
  var dateRow = Math.floor(time/86400)-18671;
  var row = getColName(dateRow)
  var rangeData = sheet.getRange(row+':'+row).getValues();
  var lastColumn = 0;
  for(var i=0; i<rangeData.length; i++){
    if(rangeData[i][0]){
      lastColumn = i+1;
    }
  }
  const range = sheet.getRange(lastColumn+1, dateRow);
  range.setValue(name);
  if(temp === "green"){
    range.setBackgroundRGB(120, 255, 120)
  }else if(temp == "yellow"){
    range.setBackgroundRGB(255, 255, 120)
  }else if (temp == "red"){
    range.setBackgroundRGB(255, 120, 120)
  }
}

function getColName(num) {
  var sheet = SpreadsheetApp.getActiveSheet();
  var result = sheet.getRange(1, num);
  result = result.getA1Notation();
  result = result.replace(/\d/,'');
 
  return result;
}

