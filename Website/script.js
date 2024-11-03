/*const nav = document.querySelector(".navbar")
const searchIcon = document.querySelector("#searchIcon")

console.log(searchIcon)

searchIcon.addEventListener("click", () => {
    nav.classList.toggle("openSearch");
    if (nav.classList.contains("openSearch")) {
        searchIcon.classList.replace("uil-search", "uil-times");
    }
    else searchIcon.classList.replace("uil-times", "uil-search");
});*/

let navLinks = document.querySelector(".links")
let menuOpenBtn = document.querySelector(".navbar .bx-menu")
let menuCloseBtn = document.querySelector(".navbar .uil-times")

menuOpenBtn.addEventListener("click", () => {
    navLinks.style.left = "0";
    menuOpenBtn.style.transform = "rotate(90deg)";
})

menuCloseBtn.addEventListener("click", () => {
    navLinks.style.left = "-100%";
    menuOpenBtn.style.transform = "rotate(0deg)";
})

var header = document.getElementById("header")
var sticky = header.offsetTop

window.onscroll = function() {
    if (window.scrollY > sticky) {
        header.classList.add("sticky")
    }
    else {
        header.classList.remove("sticky")
    }
}

