// This was ripped from rot13.org
function rot13(input) {
  var alpha="abcdefghijklmnopqrstuvwxyz";
  var newAlpha=new Array();
  for (i = 0; i < alpha.length; i++)
    newAlpha[alpha.charAt(i)] = alpha.charAt((i+13)%26);
  for (i = 0; i < alpha.length; i++)
    newAlpha[alpha.charAt(i).toUpperCase()] = alpha.charAt((i+13)%26).toUpperCase();
  var output="";
  for (i = 0; i < input.length; i++) {
    var c=input.charAt(i);
    output += ((c>= 'A' && c <= 'Z') || ( c>= 'a' && c <= 'z') ?  newAlpha[c] : c);
  }
  return output;
};

$(document).ready(function() {
  // Decrypt email address
  $('#email').attr( "href", rot13("znvygb:znry.cevzrg_ng_tznvy.pbz, yvbary.zbvfna_ng_cnevfqrfpnegrf.se"));

  // Create boxes
  $('.box').wrapInner('<div class="box_in"/>');
  $('.box').wrapInner('<div class="box_out"/>');
});


