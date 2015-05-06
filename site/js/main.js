$(document).ready(function ($) {
    console.log("JS FIRED");
    xively.setKey("MKPFAnS47P9FJAV2D7vw5M9MmHWdsEnj7zuCuJiaoyvua8jO");
    console.log("Key found. Setting JQuery");

    var feedID = 1352564954;

    xively.feed.get(feedID, function (datastream) {
        $(#dataUpdated).html(datastream["datastreams"]["0"]["current_value"]);
        $(#gpsAltitudeFt).html(datastream["datastreams"]["1"]["current_value"]);
        $(#gpsCourse).html(datastream["datastreams"]["2"]["current_value"]);
        $(#gpsLat).html(datastream["datastreams"]["3"]["current_value"]);
        $(#gpsLon).html(datastream["datastreams"]["4"]["current_value"]);
        $(#gpsSpeedMPH).html(datastream["datastreams"]["5"]["current_value"]);
        $(#hdop).html(datastream["datastreams"]["6"]["current_value"]);
        $(#satellites).html(datastream["datastreams"]["7"]["current_value"]);   

        xively.feed.subscribe(feedID, function (event, datastream_updated) {
            $(dataUpdated).html(datastream["datastreams"]["0"]["current_value"]);
        	$(gpsAltitudeFt).html(datastream["datastreams"]["1"]["current_value"]);
        	$(gpsCourse).html(datastream["datastreams"]["2"]["current_value"]);
        	$(gpsLat).html(datastream["datastreams"]["3"]["current_value"]);
        	$(gpsLon).html(datastream["datastreams"]["4"]["current_value"]);
        	$(gpsSpeedMPH).html(datastream["datastreams"]["5"]["current_value"]);
        	$(hdop).html(datastream["datastreams"]["6"]["current_value"]);
        	$(satellites).html(datastream["datastreams"]["7"]["current_value"]);   
        });
    });
});