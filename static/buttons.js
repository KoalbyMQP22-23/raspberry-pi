// addEventListener("DOMContentLoaded", function () {
//     var commandButtons = document.querySelectorAll(".command");
//     // console.log(commandButtons);
//     for (var i = 0, l = commandButtons.length; i < l; i++) {
//         var button = commandButtons[i];
//         // For each button, listen for the "click" event
//         button.addEventListener("click", function (e) {
//             // When a click happens, stop the button
//             // from submitting our form (if we have one)
//             e.preventDefault();
//
//             var clickedButton = e.target;
//             var command = clickedButton.value;
//
//             //queueElements.push(command);
//
//             // Now we need to send the data to our server
//             // without reloading the page - this is the domain of
//             // AJAX (Asynchronous JavaScript And XML)
//             // We will create a new request object
//             // and set up a handler for the response
//             var request = new XMLHttpRequest();
//             request.onload = function () {
//                 var primitive = document.createElement("p");
//                 var text = document.createTextNode(request.responseText);
//                 primitive.appendChild(text);
//                 localStorage.setItem("queue", localStorage.getItem("queue") + text)
//                 document.getElementById('queue').appendChild(primitive);
//             };
//                 // We point the request at the appropriate command
//                 request.open("GET", "/home/pre-recorded/" + command, true);
//                 // and then we send it off
//                 request.send();
//         });
//     }
//
// }, true);
//
//
// function queueView() {
//
//     var queueList = localStorage.getItem("queueList").toString().split(',')
//     for (let queueElementNum in queueList) {
//         console.log(queueList[queueElementNum])
//
//         // const upButton = document.createElement("button");
//         // upButton.id = queueList[queueElementNum] + 'upButton' + queueElementNum;
//         // upButton.innerText = '+'
//         // document.getElementById('queueBox').appendChild(upButton);
//
//         const stepHTMLElement = document.createElement("p");
//         const stepText = document.createTextNode(queueList[queueElementNum]);
//         stepHTMLElement.appendChild(stepText);
//         document.getElementById('queueBox').appendChild(stepHTMLElement);
//
//         // const downButton = document.createElement("button");
//         // downButton.id = queueList[queueElementNum] + 'downButton' + queueElementNum;
//         // downButton.innerText = '-'
//         // document.getElementById('queueBox').appendChild(downButton);
//
//     }
// }
//
