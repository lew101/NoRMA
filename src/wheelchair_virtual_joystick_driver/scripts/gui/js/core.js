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

    window.movementPublisher = new ROSLIB.Topic({
        ros : window.ros,
        name : '/cmd_vel',
        messageType : 'geometry_msgs/Twist'
    });

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
    if(linear > 10) linear = 10;
    if(linear < -10) linear = -10;
    if(angular > 10) angular = 10;
    if(angular < -10) angular = -10;

    var twist = new ROSLIB.Message({
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular}
    });
    window.movementPublisher.publish(twist);
}
