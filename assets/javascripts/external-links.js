// Open all external links in a new tab
document.addEventListener("DOMContentLoaded", function () {
  document.querySelectorAll("a[href]").forEach(function (link) {
    if (link.hostname && link.hostname !== window.location.hostname) {
      link.setAttribute("target", "_blank");
      link.setAttribute("rel", "noopener noreferrer");
    }
  });
});
