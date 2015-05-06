$(document).ready(function ($) {
    console.log("JS FIRED");
    xively.setKey("MKPFAnS47P9FJAV2D7vw5M9MmHWdsEnj7zuCuJiaoyvua8jO");
    console.log("Key found. Setting JQuery");

    var feedID = 1352564954,
            dataUpdated = "#dataUpdated",
            gpsLat = "#gpsLat",
            gpsLon = "#gpsLon",
            satellites = "#satellites",
            hdop = "#hdop",
            gpsAltitudeFt = "#gpsAltitudeFt",
            gpsSpeedMPH = "#gpsSpeedMPH",
            gpsCourse = "#gpsCourse";

    xively.feed.get(feedID, function (datastream) {
        $(dataUpdated).html(datastream["datastreams"]["dataUpdated"]["current_value"]);
        $(gpsLat).html(datastream["datastreams"]["gpsLat"]["current_value"]);
        $(gpsLon).html(datastream["datastreams"]["gpsLon"]["current_value"]);
        $(satellites).html(datastream["datastreams"]["satellites"]["current_value"]);
        $(hdop).html(datastream["datastreams"]["hdop"]["current_value"]);
        $(gpsAltitudeFt).html(datastream["datastreams"]["gpsAltitudeFt"]["current_value"]);
        $(gpsSpeedMPH).html(datastream["datastreams"]["gpsSpeedMPH"]["current_value"]);
        $(gpsCourse).html(datastream["datastreams"]["gpsCourse"]["current_value"]);

        xively.feed.subscribe(feedID, function (event, datastream_updated) {
            $(dataUpdated).html(datastream["datastreams"]["dataUpdated"]["current_value"]);
            $(gpsLat).html(datastream["datastreams"]["gpsLat"]["current_value"]);
            $(gpsLon).html(datastream["datastreams"]["gpsLon"]["current_value"]);
            $(satellites).html(datastream["datastreams"]["satellites"]["current_value"]);
            $(hdop).html(datastream["datastreams"]["hdop"]["current_value"]);
            $(gpsAltitudeFt).html(datastream["datastreams"]["gpsAltitudeFt"]["current_value"]);
            $(gpsSpeedMPH).html(datastream["datastreams"]["gpsSpeedMPH"]["current_value"]);
            $(gpsCourse).html(datastream["datastreams"]["gpsCourse"]["current_value"]);
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
