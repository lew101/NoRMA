$( document ).ready(function() {

    ws_url = 'ws://' + window.location.hostname + ':9090';

    window.ros = new ROSLIB.Ros({
        url : ws_url
    });

    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        $('.connection-status').addClass('badge-success')
        $('.connection-status').removeClass('badge-danger')
        $('.connection-status').html('Connected');
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        $('.connection-status').addClass('badge-danger')
        $('.connection-status').removeClass('badge-success')
        $('.connection-status').html('Offline');
    });

    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        $('.connection-status').addClass('badge-danger')
        $('.connection-status').removeClass('badge-success')
        $('.connection-status').html('Offline');
    });

    $('.connection-status').on('click', function(){
        window.ros.connect(ws_url);
    });

    window.speechText = new ROSLIB.Topic({
        ros : window.ros,
        name : '/speech',
        messageType : 'std_msgs/String'
    });

    window.jointsTest = new ROSLIB.Topic({
        ros : window.ros,
        name : '/reset_joints',
        messageType : 'std_msgs/Empty'
    });

    window.speechVoiceName = new ROSLIB.Topic({
        ros : window.ros,
        name : '/speech_settings/voice_name',
        messageType : 'std_msgs/String'
    });

    window.movementPublisher = new ROSLIB.Topic({
        ros : window.ros,
        name : '/cmd_vel',
        messageType : 'geometry_msgs/Twist'
    });

    window.limbMovementPublisher = new ROSLIB.Topic({
        ros : window.ros,
        name : '/move_joints',
        messageType : 'csjbot_alice/JointMovement',
        latch: true
    });

    window.limbResetPublisher = new ROSLIB.Topic({
        ros : window.ros,
        name : '/reset_joints',
        messageType : 'std_msg/Empty'
    });

    window.expressionPublisher = new ROSLIB.Topic({
        ros : window.ros,
        name : '/expression',
        messageType : 'csjbot_alice/Expression'
    });

    window.videoSubscriber = new ROSLIB.Topic({
        ros : window.ros,
        name : '/video_on',
        messageType : 'std_msgs/Bool'
    });
 
    window.partsListUpdatedSubscriber = new ROSLIB.Topic({
        ros : window.ros,
        name : '/alice/parts/updated',
        messageType : 'csjbot_alice/PartsListUpdate'
    });

    window.scanResultSubscriber = new ROSLIB.Topic({
        ros : window.ros,
        name : '/alice/parts/scan_result',
        messageType : 'std_msgs/String'
    });

    window.partTransferPublisher = new ROSLIB.Topic({
        ros : window.ros,
        name : '/alice/parts/transfer',
        messageType : 'csjbot_alice/PartTransfer'
    });

    window.parts_workshop = new ROSLIB.Param({
        ros : window.ros,
        name :  '/alice/parts/workshop'
    });

    window.parts_requested = new ROSLIB.Param({
        ros : window.ros,
        name :  '/alice/parts/requested'
    });

    window.parts_intransit = new ROSLIB.Param({
        ros : window.ros,
        name :  '/alice/parts/intransit'
    });

    window.parts_warehouse = new ROSLIB.Param({
        ros : window.ros,
        name :  '/alice/parts/warehouse'
    });

    window.partsListUpdatedSubscriber.subscribe(function(message) {
        console.log("received parts list update " + message.location_name);
        update_parts_list(message.location_name);
    });
    
    $(document).on('click', '.btn-parts-action', function(){
        source = $(this).parents('.parts-list').data("type");
        target = $(this).data('btntype');
        id = $(this).parents('.part').data('id');
        
        window.partTransferPublisher.publish(
            new ROSLIB.Message({
                id: id,
                location_source: source,
                location_target: target,
                trigger_update: true,
            })
        );
    });

    window.openHabStatePublisher = new ROSLIB.Topic({
        ros : window.ros,
        name : '/alice/openhab/state/',
        messageType : 'std_msgs/Bool'
    });

    $(document).on('change', '.openhab-state-switch', function(){ 
        window.openHabStatePublisher.publish(new ROSLIB.Message({data:$(this).is(':checked')}));
    });

    
    window.requestScan = new ROSLIB.Topic({
        ros : window.ros,
        name : '/alice/parts/scan',
        messageType : 'std_msgs/Empty'
    });

    $(document).on('click', '.btn-scan-tray', function(){
        $('.scan-loading').removeClass('d-none');
        $('#intransit .list-group').addClass('d-none');
        $('.inferred-frame').addClass('d-none');
        $('.scan-result-list ul').empty();
        window.requestScan.publish();
    });

    $(document).on('click', '.btn-confirm-requested', function(){
        var parts_speech = "";
        $('#requested').find('li').each(function() {
            parts_speech += $(this).find('.parts-qty').html() + " " + $(this).find('.parts-name').html() + ". ";
        });  
        if (parts_speech == '') {
            parts_speech = 'No parts have been requested.';
        } else {
            parts_speech = "The following parts have been requested. " + parts_speech;
        }

        speak(parts_speech);
    });

    $(document).on('click', '.btn-confirm-parts', function(){
        var parts_speech = "";
        $('#intransit').find('li').each(function() {
            parts_speech += $(this).find('.parts-qty').html() + " " + $(this).find('.parts-name').html() + ". ";
        });  
        if (parts_speech == '') {
            parts_speech = 'My tray is empty. I have no parts.';
        } else {
            parts_speech = "I have the following parts on my tray. " + parts_speech;
        }

        speak(parts_speech);
    });

    window.videoSubscriber.subscribe(function(message) {
        console.log('Received message on ' + window.videoSubscriber.name + ': ' + message.data);
        if (message.data == true) {
            $('.visuals').removeClass('alert-danger');
            $('.visuals').addClass('alert alert-success');
        } else {
            $('.visuals').removeClass('alert-success');
            $('.visuals').addClass('alert alert-danger');
        }
    });

    window.scanResultSubscriber.subscribe(function(message) {
        console.log('Received message on ' + window.scanResultSubscriber.name + ': ' + message.data);
        $('.scan-result-list ul').append('<li>' + message.data + '</li>');
    });

    $('.publish-speech').on('click', function(){
        speak($('.custom-text-to-speak').val());
        $('.custom-text-to-speak').val('');
    });

    $(document).on('click', '.quick-speak, .long-speak', function(){
        speak($(this).data('phrase'));
    });

    $(document).on('click', '.cancel-speech', function() {
        speak('');
    });

    $('.movement').on('click', function() {
        move($(this).data('linear'), $(this).data('angular'));
    });

    $(".voice-option").click(function(){
        change_voice($(this).data('voice'));
    });

    $('input[type=range]').on("change", $.throttle(250, function() {
        moveLimbs($(this).data('limb'));
    }));

    $('.link-arms').on('click', function(){
        if ($(this).data('state') == "off") {
            $(this).removeClass('btn-secondary');
            $(this).addClass('btn-success');
            $(this).data('state', 'on');
        } else {
            $(this).removeClass('btn-success');
            $(this).addClass('btn-secondary');
            $(this).data('state', 'off');
        }
    });

    $('.reset-limbs').on('click', function() {
        $('.appendix-movement').val(0);
        window.limbResetPublisher.publish();
    });

    $('.quick-expression').on('click', function(){
        window.expressionPublisher.publish(new ROSLIB.Message({data:$(this).data('expression')}));
    });

    $(document).on('click', '.context', function(){
        // Clear up UI in prep
        $('.context').removeClass('active');
        $(this).addClass('active');

        $('.panel').addClass('d-none');

        context = $(this).data('id');
        var contexts_details = new ROSLIB.Param({
            ros : ros,
            name :  '/alice/contexts/' + context
        });
    
        contexts_details.get(function(value){
            $.each(value, function( index, item ){
                if (index == 'panels') {
                    $.each(item, function( panelid, panel ){
                        $('.panel-' + panel).removeClass('d-none');
                    });
                }
            });
        });

        retrieve_speechset(context); 
    });

    retrieve_contexts();

    update_parts_list('warehouse');
    update_parts_list('requested');
    update_parts_list('intransit');
    update_parts_list('workshop');

    change_voice($('.voice-default').data('voice'));
    init_video_stream();
});

function retrieve_contexts() {
    var contexts_param = new ROSLIB.Param({
        ros : ros,
        name :  '/alice/contexts/'
    });

    $('.contexts').html("");
    contexts_param.get(function(value){
        $.each(value, function( index, item ){
            context_option = $('.templates .context').clone();
            context_option.data('id', index);
            context_option.html(item.name);
            context_option.appendTo(".contexts");
        });
        $('.contexts .context').first().trigger('click');
    });
}

function retrieve_speechset(set) {
    var speechset_short_param = new ROSLIB.Param({
        ros : ros,
        name :  '/alice/speechsets/' + set
    });

    $('.speech-options-long, .speech-options-short').html("");
    speechset_short_param.get(function(value){
        $.each(value, function( index, item ){
            $.each(item, function( name, speechset ){
                if (name == "short") {
                    $.each(speechset, function(i, phrase){
                        $( ".templates .quick-speak" ).clone().html(phrase).data('phrase', phrase).appendTo(".speech-options-short");
                    });
                }
                if (name == "long") {
                    $.each(speechset, function(i, phrase){
                        display_phrase = phrase;
                        if (phrase.length > 60) {
                            display_phrase = phrase.substring(0, 60) + "...";
                        }
                        $( ".templates .long-speak" ).clone().html(display_phrase).data('phrase', phrase).appendTo(".speech-options-long");
                    });
                }
            });
        });
    });
}

function update_parts_list(location) {
    if (location == 'warehouse'){
        location_param = parts_warehouse;
    }
    if (location == 'requested'){
        location_param = parts_requested;
    }
    if (location == 'intransit'){
        location_param = parts_intransit;
        $('#intransit .list-group').removeClass('d-none');
        $('.scan-loading').addClass('d-none');
        filename = 'media/inferred.jpg?' + Math.random();
        $('.inferred-frame img').attr('src', filename);
        $('.inferred-frame').removeClass('d-none');
    }
    if (location == 'workshop'){
        location_param = parts_workshop;
    }
    retrieve_parts(location_param, location);
}

function retrieve_parts(location_param, location) {
    $('#' + location).find('ul').empty();
    location_param.get(function(value){
        console.log(value);
        $.each(value, function( index, part ){
            part_obj = $( ".templates .part" ).clone()
            part_obj.find('.parts-name').html(part.name)
            part_obj.find('.parts-qty').html(part.qty)
            part_obj.find('.parts-img').attr("src", "media/parts/" + part.id + ".png")
            part_obj.data('id', part.id)
            part_obj.data('model', part.model)
            part_obj.data('qty', part.qty)
            part_obj.find('[data-btntype="' + location + '"]').addClass('disabled')
            part_obj.appendTo( "#" + location + " ul" );
        });
        update_counts();
    });
}

function update_counts() {
    $('.count').each(function(){
        count = 0;
        lis = $('#' + $(this).data('type') + ' ul').find('li');
        lis.each(function(){
            count += $(this).data('qty');
        });
        $(this).html(count);
    });
}

function speak(message) {
    window.speechText.publish(new ROSLIB.Message({data:message}));
}

function change_voice(voicename) {
    window.speechVoiceName.publish(new ROSLIB.Message({data:voicename}))
}

function moveLimbs(limb_to_move) {
    neck_move = false;
    left_arm_move = false;
    right_arm_move = false;

    neck_pos = parseInt(-$('.neck').val());
    left_arm_pos = parseInt($('.left-arm').val());
    right_arm_pos = parseInt($('.right-arm').val());

    switch (limb_to_move){
        case 'neck':
            neck_move = true;
            break;

        case 'left_arm':
            left_arm_move = true;
            if ($('.link-arms').data('state') == "on") {
                right_arm_move = true;
                right_arm_pos = left_arm_pos;
                $('.right-arm').val(left_arm_pos)
            }
            break;

        case 'right_arm':
            right_arm_move = true;
            if ($('.link-arms').data('state') == "on") {
                left_arm_move = true;
                left_arm_pos = right_arm_pos;
                $('.left-arm').val(right_arm_pos)
            }
            break;

    }

    var joint_movement = new ROSLIB.Message({
        neck: neck_move,
        neck_to: neck_pos,
        neck_speed: 5000,
        left_arm: left_arm_move,
        left_arm_to: left_arm_pos,
        left_arm_speed: 3000,
        right_arm: right_arm_move,
        right_arm_to: right_arm_pos,
        right_arm_speed: 3000
    });

    window.limbMovementPublisher.publish(joint_movement);
}

function move(linear, angular) {
    if(linear > 0.5) linear = 0.5;
    if(linear < -0.5) linear = -0.5;
    if(angular > 1) angular = 1;
    if(angular < -1) angular = -1;

    var twist = new ROSLIB.Message({
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular}
    });
    window.movementPublisher.publish(twist);
}

function init_video_stream() {
    var viewer = new MJPEGCANVAS.Viewer({
        divID : 'mjpeg',
        host : window.location.hostname,
        width : 320,
        height : 240,
        topic : '/camera/image'
    });
}