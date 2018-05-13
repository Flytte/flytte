/**
 * Exception class for the cases when a box can not be found.
 */
class BoxNotFoundException
{
    constructor(id, message = null)
    {
        this.id = id;

        if(message === null)
        {
            this.message = `A box with the id '#box${id}' could not be found`;
        }
        else
        {
            this.message = message;
        }
    }
}

/**
 * Exception class for the cases of an invalid argument.
 */
class InvalidArgumentException
{
    constructor(func, arg, val, message = null)
    {
        this.func = func;
        this.arg = arg;
        this.val = val;

        if(message === null)
        {
            this.message = `Invalid value ${val} of the argument ${arg} in ${func}`;
        }
        else
        {
            this.message = message;
        }
    }
}

/**
 * Exception class for the cases when element already exists and something tries to override it.
 */
class ElementExistsException
{
    constructor(elem, message = null)
    {
        this.elem = elem;

        if(message === null)
        {
            this.message = `Element ${elem} already exists`;
        }
        else
        {
            this.message = message;
        }
    }
}



class AbstractBox
{
    constructor(name, posX, posY, width, height, rotX, rotY, rotZ)
    {
        // Dirty hack. Setting size of the container equal to the size of the video  
        $("#content")[0].style.width = $('.video')[0].width + "px";
        $("#content")[0].style.height = $('.video')[0].height + "px";
    
        // Creating a box with the given size and position
        let box = $("<div></div>");
        box.css({
            "left": 0 + "%",
            "top": 0 + "%",
            "width": 0 + "%",
            "height": 0 + "%"
        });

        // Creating a label for the box
        let label = $("<div></div>");
        label.addClass("box-label");
        label.html(`
            <p class='box-label-name'>
                <span class='box-name'></span>
            </p>
            <p class='box-label-position'>
                Position:
                [
                <span class='box-position-x'></span>,
                <span class='box-position-y'></span>
                ]
            </p>
            <p class='box-label-rotation'>
                Rotation:
                [
                <span class='box-rotation-x'></span>,
                <span class='box-rotation-y'></span>,
                <span class='box-rotation-z'></span>
                ]
            </p>
        `);

        box.append(label);

        this._view = box;
        this._container = $("#content");
        
        this.name = name;
        this.posX = posX;
        this.posY = posY;
        this.width = width;
        this.height = height;
        this.rotX = rotX;
        this.rotY = rotY;
        this.rotZ = rotZ;

        this._container.append(this._view);
    }

    destruct()
    {
        this._view.remove();
    }


    get name()
    {
        return this._view.find(".box-name").text();
    }

    set name(name)
    {
        this._view.find(".box-name").text(name);
    }


    get posX()
    {
        return parseFloat(this._view.find(".box-position-x").text());
    }

    set posX(posX)
    {
        this._view.find(".box-position-x").text(posX.toFixed(2));
        this._view.css("left", posX + "%");
    }


    get posY()
    {
        return parseFloat(this._view.find(".box-position-y").text());
    }

    set posY(posY)
    {
        this._view.find(".box-position-y").text(posY.toFixed(2));
        this._view.css("top", posY + "%");
    }


    get width()
    {
        return parseFloat(this._view.css("width")) / parseFloat(this._container.css("width")) * 100;
    }

    set width(width)
    {
        this._view.css("width", width + "%");
    }


    get height()
    {
        return parseFloat(this._view.css("height")) / parseFloat(this._container.css("height")) * 100;
    }

    set height(height)
    {
        this._view.css("height", height + "%");
    }


    get rotX()
    {
        return parseFloat(this._view.find(".box-rotation-x").text());
    }

    set rotX(rotX)
    {
        this._view.find(".box-rotation-x").text(this._normalizeRotation(rotX).toFixed(2));
    }


    get rotY()
    {
        return parseFloat(this._view.find(".box-rotation-y").text());
    }

    set rotY(rotY)
    {
        this._view.find(".box-rotation-y").text(this._normalizeRotation(rotY).toFixed(2));
    }


    get rotZ()
    {
        return parseFloat(this._view.find(".box-rotation-z").text());
    }

    set rotZ(rotZ)
    {
        this._view.find(".box-rotation-z").text(this._normalizeRotation(rotZ).toFixed(2));
    }
    
    
    _normalizeRotation(val)
    {
        return val % 360;
    }
}



class Box extends AbstractBox
{
    constructor(id, name, posX, posY, width, height, rotX, rotY, rotZ)
    {
        super(name, posX, posY, width, height, rotX, rotY, rotZ);

        this._id = id;
        this._target = null;

        this._view.addClass("box");
        this._view.attr("id", "box" + id);

        let self = this;
        
        this._view.draggable({
            containment: "#content",
            cursor: "move",
            cancel: ".box-label",
            helper: "clone",
            stop: function(event, ui)
            {
                let content = $("#content");

                let x = Math.round(ui.position.left / (content.width() / 100));
                let y = Math.round(ui.position.top / (content.height() / 100));

                if(self.hasTarget())
                {
                    self.removeTarget();
                }

                self.createTarget(x, y, self.width, self.height, self.rotX, self.rotY, self.rotZ);
            }
        });
        
        this._view.on("wheel", function(e)
        {
            let rotZ = self.rotZ;
            let width = self.width;
            let height = self.height;
        
            if($(document).prop("_ctrl"))
            {
                rotZ += e.deltaY;
            }
            else
            {
                let prop = height / width;
                
                width += e.deltaY;
                height = width * prop;
            }
            
            if(self.hasTarget())
            {
                self.removeTarget();
            }
            
            self.createTarget(self.posX, self.posY, width, height, self.rotX, self.rotY, rotZ);
            
            e.preventDefault();
        });
    }

    destruct()
    {
        super.destruct();
        this.removeTarget();
    }

    createTarget(posX, posY, width, height, rotX, rotY, rotZ)
    {
        this._target = new Target(this, posX, posY, width, height, rotX, rotY, rotZ);
    }

    removeTarget()
    {
        if(this._target === null)
        {
            return;
        }

        this._target.destruct();
        this._target = null;
    }

    hasTarget()
    {
        return this._target !== null;
    }

    get id()
    {
        return this._id;
    }

    get target()
    {
        return this._target;
    }
}



class Target extends AbstractBox
{
    constructor(box, posX, posY, width, height, rotX, rotY, rotZ)
    {
        super(box.name, posX, posY, width, height, rotX, rotY, rotZ);

        this._id = box.id;
        this._box = box;

        this._view.addClass("target");
        this._view.attr("id", "target" + this._id);

        let self = this;
        
        this._view.draggable({
            containment: "#content",
            cursor: "move",
            cancel: ".box-label",
            stop: function(event, ui)
            {
                let content = $("#content");

                self.posX = Math.round(ui.position.left / (content.width() / 100));
                self.posY =  Math.round(ui.position.top / (content.height() / 100));

                // Send new position of the target to the backend
                let msg = new ROSLIB.Message({
                    id : self.id,
                    t :
                        {
                            linear :
                                {
                                    x : self.posX,
                                    y : self.posY,
                                    z : 0
                                },
                            angular :
                                {
                                    x : self.rotX,
                                    y : self.rotY,
                                    z : self.rotZ
                                }
                        },
                    w : self.width,
                    h : self.height
                });

                backend_out_commands.publish(msg);
                console.log(msg);
            }
        });
        
        this._view.on("wheel", function(e)
        {
            if($(document).prop("_ctrl"))
            {
                self.rotZ += e.deltaY;
            }
            else
            {
                let prop = self.height / self.width;
                
                self.width += e.deltaY;
                self.height = self.width * prop;
            }
            
            let msg = new ROSLIB.Message({
                id : self.id,
                t :
                    {
                        linear :
                            {
                                x : self.posX,
                                y : self.posY,
                                z : 0
                            },
                        angular :
                            {
                                x : self.rotX,
                                y : self.rotY,
                                z : self.rotZ
                            }
                    },
                w : self.width,
                h : self.height
            });

            backend_out_commands.publish(msg);
            console.log(msg);
            
            e.preventDefault();
        });

        let msg = new ROSLIB.Message({
            t :
                {
                    linear :
                        {
                            x : this.posX,
                            y : this.posY,
                            z : 0
                        },
                    angular :
                        {
                            x : this.rotX,
                            y : this.rotY,
                            z : this.rotZ
                        }
                },
            w : this.width,
            h : this.height
        });

        backend_out_commands.publish(msg);
        console.log(msg);
    }

    get box()
    {
        return this._box;
    }
}



class BoxPool
{
    constructor()
    {
        this._pool = new Map;
    }

    createBox(id, name, posX, posY, width, height, rotX, rotY, rotZ)
    {
        if(id === null)
        {
            throw new InvalidArgumentException("createBox(...)", "id", "null");
        }

        if(this.hasBox(id))
        {
            throw new ElementExistsException("box" + id);
        }

        let box = new Box(id, name, posX, posY, width, height, rotX, rotY, rotZ);

        this._pool.set(id, box);
    }

    removeBox(id)
    {
        if(this._pool.has(id))
        {
            let box = this._pool.get(id);

            box.destruct();
            this._pool.delete(id);
        }
    }

    hasBox(id)
    {
        return this._pool.has(id);
    }

    findBox(id)
    {
        return this._pool.get(id);
    }

    size()
    {
        return this._pool.size;
    }

    keys()
    {
        return Array.from(this._pool.keys());
    }
}



class InputHandler
{
    constructor(pool)
    {
        this._pool = pool;
    }

    handle(data)
    {
        let dronesLost = this._pool.keys();

        for(let index in data.drones)
        {
            let drone = data.drones[index];

            if(this._pool.hasBox(drone.id))
            {
                // If the box exists already just update the data
                let box = this._pool.findBox(drone.id);

                box.posX = drone.box.t.linear.x;
                box.posY = drone.box.t.linear.y;
                box.width = drone.box.w;
                box.height = drone.box.h;
                box.rotX = drone.box.t.angular.x;
                box.rotY = drone.box.t.angular.y;
                box.rotZ = drone.box.t.angular.z;

                // Remove it from the list of boxes that will be removed
                let index = dronesLost.indexOf(drone.id);
                dronesLost.splice(index, 1);
            }
            else
            {
                // If the box does not exist yet create it
                this._pool.createBox(
                    drone.id,
                    drone.name,
                    drone.box.t.linear.x,
                    drone.box.t.linear.y,
                    drone.box.w,
                    drone.box.h,
                    drone.box.t.angular.x,
                    drone.box.t.angular.y,
                    drone.box.t.angular.z)
            }

            // Remove all the boxes that are not in the last message
            for(let id in dronesLost)
            {
                this._pool.removeBox(id);
            }
        }
    }
}



/**
 * Creates an alert message on the screen.
 *
 * @param type type of the alert, should be one of the bootstrap alert types
 * @param msg message of the alert
 */
function createAlert(type, msg)
{
    let alert = $("<div></div>");
    alert.addClass("alert alert-dismissable fade in ui-element");
    alert.addClass(type);
    alert.html("<strong>Message: </strong>" + msg);

    let button = $("<button></button>");
    button.addClass("close");
    button.attr("data-dismiss", "alert");
    button.html("&times");

    alert.append(button);

    alert.dblclick(function()
    {
        $(this).alert("close");
    });

    $("#content").append(alert);
}

/**
 * Boolean that defines whether client works in a simulation mode.
 * In this mode no web camera is used and no frames are being sent to the backend,
 * instead video stream is being received from the server.
 */
let simulation = false;

/**
 * ROS Publisher object for drone commands.
 * Command messages for the drones have to be published here.
 */
let backend_out_commands;

/**
 * ROS Publisher object for video frames.
 */
let backend_out_frames;

/**
 * ROS Subscriber object.
 * List of the drones recognized by the backend will come through here.
 */
let backend_in;

/**
 * Main function. This is being executed when the DOM of the page is loaded.
 */
$(document).ready(function()
{
    // ==================================  INIT  ================================================ //

    let pool = new BoxPool();
    let inputHandler = new InputHandler(pool);
    
    // ================================  CTRL KEY  ============================================== //
    
    $(document).prop("_ctrl", false);
    
    $(document).keydown(
        function(e)
        {
            if(e.which == 17)
            {
                $(document).prop("_ctrl", true);
            }
        }
    );
    
    $(document).keyup(
        function(e)
        {
            if(e.which == 17)
            {
                $(document).prop("_ctrl", false);
            }
        }
    );


    // ==================================  ROS  ================================================ //

    let ros = new ROSLIB.Ros();

    ros.on('error', function(error) {
        console.log("ROS Error: ", error);
        createAlert("alert-danger", "ROS Error");
    });

    ros.on('connection', function() {
        console.log("ROS connection established");
        createAlert("alert-success", "ROS connection established");
    });

    ros.on('close', function() {
        console.log("ROS connection closed");
        createAlert("alert-danger", "ROS connection closed");
    });

    ros.connect('ws://localhost:9090');

    backend_out_commands = new ROSLIB.Topic({
        ros : ros,
        name : '/commands',
        messageType : 'drone_app_msgs/BBox'
    });

    backend_out_frames = new ROSLIB.Topic({
        ros : ros,
        name : '/frames',
        messageType : 'std_msgs/String'
    });

    backend_in = new ROSLIB.Topic({
        ros : ros,
        name : '/client_drones',
        messageType : 'drone_app_msgs/DroneArray'
    });

    backend_in.subscribe(function(message) {
        console.log('Received message on ' + backend_in.name + ': ', message);

        inputHandler.handle(message);
    });
    
    
    // ==================================  SIMULATION OR NOT  ================================================ //

    let items = location.search.substr(1).split("&");

    for(let i = 0; i < items.length; i++)
    {
            let tmp = [];
            tmp = items[i].split("=");
            
            if(tmp[0] === "simulation")
            {
                simulation = true;
                break;
            }
    }
    
    $("#content").css("overflow", "hidden");
    
    if(simulation)
    {
        $("#video-wrapper").append($('<img class="video"></img>'));
        
        let video = $('.video')[0];
        
        let src = "http://localhost:8080/stream?topic=/camera_img";
        
        video.src = src;
    }
    else
    {
        $("#video-wrapper").append($('<video class="video" autoplay muted loop></video>'));

        // ==================================  FRAMES  ================================================ //
        
        let canvas = document.createElement('canvas');
        let ctx;
        let timer;

        let timerCallback = function()
        {
            ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

            let frame = canvas.toDataURL('image/png');

            let msg = new ROSLIB.Message({
                data: frame
            });
            
            backend_out_frames.publish(msg);
        };

        let video = $('.video')[0];

        video.onloadeddata = function(e)
        {
            if(ctx)
            {
                return;
            }

            canvas.width = 256;
            canvas.height = 192;

            ctx = canvas.getContext('2d');

            timer = setInterval(timerCallback, 250);
            
            $("#content")[0].style.width = video.clientWidth + "px";
            $("#content")[0].style.height = video.clientHeight + "px";
        };
    

        // ==================================  CAMERA  ================================================ //

        // TODO Maybe remove constraints at all, they seem not to work (at least in Firefox)
        let constraints =
            {
                video:
                    {
                        width: 1024,
                        height: 768
                    },
                frameRate:
                    {
                        ideal: 10,
                        max: 10
                    }
            };

        navigator.mediaDevices.getUserMedia(constraints)
            .then(function(stream)
            {
                // Older browsers may not have srcObject
                if ("srcObject" in video)
                {
                    video.srcObject = stream;
                }
                else
                {
                    video.attr("src", window.URL.createObjectURL(stream));
                }
            })
            .catch(function(err)
            {
                let msg;

                switch(err.name)
                {
                    case 'NotFoundError':
                        msg = "Camera could not be found";
                        break;
                    case 'NotAllowedError':
                        msg = "Access to the camera has been denied";
                        break;
                    case 'NotReadableError':
                        msg = "It looks like a hardware error occurred and the camera stopped working";
                        break;
                    case 'OverconstrainedError':
                        msg = "The camera is not suitable: " + err.message;
                        break;
                    default:
                        msg = "Something went wrong with the camera (" + err.name + "): " + err.message;
                        break;
                }

                console.log(msg);
                createAlert("alert-danger", msg);

                video.src = "video/fallback.mp4";
            });
    }
});
