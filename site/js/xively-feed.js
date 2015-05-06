$(document).ready(function ($) {
    console.log("JS FIRED");
    xively.setKey("380p3cxLhGBH2Thg1H7doL2zcaoQLCDovFav2C43epC1UMvx");
    console.log("Key found. Setting JQuery");

    var feedID = 69373229,
            door = "#xivDoor",
            humidity = "#xivHumidity",
            light = "#xivLight",
            smoke = "#xivSmoke",
            tempF = "#xivTempF";

    xively.feed.get(feedID, function (datastream) {
        $(door).html(datastream["datastreams"]["0"]["current_value"]);
        $(humidity).html(datastream["datastreams"]["1"]["current_value"]);
        $(light).html(datastream["datastreams"]["2"]["current_value"]);
        $(smoke).html(datastream["datastreams"]["3"]["current_value"]);
        $(tempF).html(datastream["datastreams"]["4"]["current_value"]);

        xively.feed.subscribe(feedID, function (event, datastream_updated) {
            $(door).html(datastream["datastreams"]["0"]["current_value"]);
            $(humidity).html(datastream["datastreams"]["1"]["current_value"]);
            $(light).html(datastream["datastreams"]["2"]["current_value"]);
            $(smoke).html(datastream["datastreams"]["3"]["current_value"]);
            $(tempF).html(datastream["datastreams"]["4"]["current_value"]);
        });
    });

    var update = new Date();
    var timeString = update.toTimeString();
    var dateString = update.toDateString();
    var time = "";
    var date = "";
    for (var i = 0; i < 8; i++) {
        time += timeString.charAt(i);
    }
    for (var j = 0; j < 10; j++) {
        date += dateString.charAt(j);
    }
    var el = document.getElementById('xivLastUpdate');
    el.innerHTML = time + '<br>' + date;
});
