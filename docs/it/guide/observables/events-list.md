---
description: "Un elenco di eventi JavaScript che possono essere usati con fromEvent in RxJS, organizzati per tipo. Fornisce informazioni di riferimento per categoria per implementare la gestione di eventi come mouse, tastiera, form, touch, drag & drop, media, window, ecc."
---
# Elenco degli eventi

## 1. Eventi del mouse

| Nome evento JavaScript | Attributo HTML | Tipo | Descrizione | Disponibilità con fromEvent |
|---|---|---|---|---|
| click | onclick | MouseEvent | Quando viene fatto clic su un elemento | ✅ |
| dblclick | ondblclick | MouseEvent | Quando un elemento viene cliccato due volte | ✅ |
| mousedown | onmousedown | MouseEvent | Quando viene premuto un pulsante del mouse | ✅ |
| mouseup | onmouseup | MouseEvent | Quando il pulsante del mouse viene rilasciato | ✅ |
| mousemove | onmousemove | MouseEvent | Quando il mouse viene spostato | ✅ |
| mouseover | onmouseover | MouseEvent | Quando il mouse passa sopra un elemento | ✅ |
| mouseout | onmouseout | MouseEvent | Quando il mouse si allontana dall'elemento | ✅ |
| mouseenter | onmouseenter | MouseEvent | Quando il mouse entra nell'elemento (senza bubbling) | ✅ |
| mouseleave | onmouseleave | MouseEvent | Quando il mouse abbandona l'elemento (senza bubbling) | ✅ |
| contextmenu | oncontextmenu | MouseEvent | Quando viene aperto il menu del tasto destro del mouse | ✅ |


## 2. Eventi del puntatore

| Nome evento JavaScript | Attributo HTML | Tipo | Descrizione | Disponibilità con fromEvent |
|---|---|---|---|---|
| pointerdown | onpointerdown | PointerEvent | Quando viene premuto il puntatore (touch, penna, mouse) | ✅ |
| pointerup | onpointerup | PointerEvent | Quando il puntatore viene rilasciato | ✅ |
| pointermove | onpointermove | PointerEvent | Quando il puntatore viene spostato | ✅ |
| pointerover | onpointerover | PointerEvent | Quando il puntatore passa sopra un elemento | ✅ |
| pointerout | onpointerout | PointerEvent | Quando il puntatore esce dall'elemento | ✅ |
| pointerenter | onpointerenter | PointerEvent | Quando il puntatore entra in un elemento (senza bubbling) | ✅ |
| pointerleave | onpointerleave | PointerEvent | Quando il puntatore abbandona l'elemento (senza bubbling) | ✅ |
| pointercancel | onpointercancel | PointerEvent | Quando viene annullata un'operazione del puntatore | ✅ |
| gotpointercapture | ongotpointercapture | PointerEvent | Quando viene acquisita una cattura del puntatore | ✅ |
| lostpointercapture | onlostpointercapture | PointerEvent | Quando la cattura di un puntatore è persa | ✅ |


## 3. Eventi touch

| Nome evento JavaScript | Attributo HTML | Tipo | Descrizione | Disponibilità con fromEvent |
|---|---|---|---|---|
| touchstart | ontouchstart | TouchEvent | Quando lo schermo viene toccato | ✅ |
| touchmove | ontouchmove | TouchEvent | Quando il dito che tocca si muove | ✅ |
| touchend | ontouchend | TouchEvent | Quando il tocco termina | ✅ |
| touchcancel | ontouchcancel | TouchEvent | Quando un tocco viene annullato | ✅ |


## 4. Eventi della tastiera

| Nome evento JavaScript | Attributo HTML | Tipo | Descrizione | Disponibilità con fromEvent |
|---|---|---|---|---|
| keydown | onkeydown | KeyboardEvent | Quando viene premuto un tasto | ✅ |
| keypress | onkeypress | KeyboardEvent | ⚠️ **Deprecato** - usare `keydown` | ✅ |
| keyup | onkeyup | KeyboardEvent | Quando un tasto viene rilasciato | ✅ |

::: warning Informazioni sull'evento keypress
L'evento `keypress` è **deprecato** dallo standard web.

**Motivi della deprecazione**:
- Supporto insufficiente per l'internazionalizzazione (ad es. problemi con l'input in giapponese)
- Comportamento instabile in combinazione con i tasti modificatori (Shift, Ctrl, Alt)
- Supporto limitato sui dispositivi mobili

**Alternative consigliate**:
```typescript
// ❌ Deprecato
fromEvent(input, 'keypress')
  .subscribe(event => console.log(event));

// ✅ Consigliato: usare keydown
fromEvent<KeyboardEvent>(input, 'keydown')
  .subscribe(event => console.log(event.key));
```

**Eventi consigliati per caso d'uso**:
- Rilevamento dell'input di testo: evento `input` (consigliato)
- Rilevamento della pressione dei tasti: evento `keydown`
- Rilevamento del rilascio di un tasto: evento `keyup`
:::


## 5. Eventi relativi al focus

| Nome evento JavaScript | Attributo HTML | Tipo | Descrizione | Disponibilità con fromEvent |
|---|---|---|---|---|
| focus | onfocus | FocusEvent | Quando un elemento riceve il focus | ✅ |
| blur | onblur | FocusEvent | Quando il focus viene rimosso da un elemento | ✅ |
| focusin | onfocusin | FocusEvent | Quando un elemento o un elemento figlio riceve il focus | ✅ |
| focusout | onfocusout | FocusEvent | Quando un elemento o un elemento figlio perde il focus | ✅ |


## 6. Eventi del form

| Nome evento JavaScript | Attributo HTML | Tipo | Descrizione | Disponibilità con fromEvent |
|---|---|---|---|---|
| change | onchange | Event | Quando il contenuto dell'input viene cambiato | ✅ |
| input | oninput | InputEvent | Quando il valore di un campo di input viene modificato | ✅ |
| submit | onsubmit | SubmitEvent | Quando viene inviato un form | ✅ |
| reset | onreset | Event | Quando un form viene reimpostato | ✅ |
| select | onselect | Event | Quando il testo viene selezionato | ✅ |


## 7. Eventi drag & drop

| Nome evento JavaScript | Attributo HTML | Tipo | Descrizione | Disponibilità con fromEvent |
|---|---|---|---|---|
| drag | ondrag | DragEvent | Mentre un elemento viene trascinato | ✅ |
| dragstart | ondragstart | DragEvent | Quando viene avviato un trascinamento | ✅ |
| dragend | ondragend | DragEvent | Quando il trascinamento termina | ✅ |
| dragover | ondragover | DragEvent | Quando l'elemento trascinato è sopra un altro elemento | ✅ |
| dragenter | ondragenter | DragEvent | Quando l'elemento trascinato entra nel target | ✅ |
| dragleave | ondragleave | DragEvent | Quando l'elemento trascinato esce dal target | ✅ |
| drop | ondrop | DragEvent | Quando l'elemento trascinato viene rilasciato | ✅ |


## 8. Eventi finestra e documento

| Nome evento JavaScript | Attributo HTML | Tipo | Descrizione | Disponibilità con fromEvent |
|---|---|---|---|---|
| load | onload | Event | Quando la pagina è completamente caricata | ✅ |
| resize | onresize | UIEvent | Quando la dimensione della finestra viene modificata | ✅ |
| scroll | onscroll | Event | Quando la pagina viene scrollata | ✅ |
| unload | onunload | Event | Quando la pagina viene chiusa | ❌ |
| beforeunload | onbeforeunload | BeforeUnloadEvent | Appena prima che la pagina venga chiusa | ❌ |
| error | onerror | ErrorEvent | Quando si verifica un errore | ✅ |
| visibilitychange | onvisibilitychange | Event | Quando cambia lo stato di visualizzazione della pagina (es. cambio di scheda) | ✅ |
| fullscreenchange | onfullscreenchange | Event | Quando cambia lo stato di schermo intero | ✅ |


## 9. Eventi multimediali

| Nome evento JavaScript | Attributo HTML | Tipo | Descrizione | Disponibilità con fromEvent |
|---|---|---|---|---|
| play | onplay | Event | Quando inizia la riproduzione multimediale | ✅ |
| pause | onpause | Event | Quando la riproduzione multimediale è in pausa | ✅ |
| ended | onended | Event | Quando la riproduzione multimediale termina | ✅ |
| volumechange | onvolumechange | Event | Quando il volume del media viene modificato | ✅ |
| seeking | onseeking | Event | Quando la ricerca del media è iniziata | ✅ |
| seeked | onseeked | Event | Quando la ricerca del media è completata | ✅ |
| timeupdate | ontimeupdate | Event | Quando il tempo di riproduzione del media viene aggiornato | ✅ |


## 10. Eventi di animazione e transizione

| Nome evento JavaScript | Attributo HTML | Tipo | Descrizione | Disponibilità con fromEvent |
|---|---|---|---|---|
| animationstart | onanimationstart | AnimationEvent | Quando inizia l'animazione | ✅ |
| animationend | onanimationend | AnimationEvent | Quando l'animazione finisce | ✅ |
| animationiteration | onanimationiteration | AnimationEvent | Quando l'animazione viene ripetuta | ✅ |
| transitionstart | ontransitionstart | TransitionEvent | Quando viene avviata una transizione CSS | ✅ |
| transitionend | ontransitionend | TransitionEvent | Quando una transizione CSS termina | ✅ |


## 11. Altri eventi

| Nome evento JavaScript | Attributo HTML | Tipo | Descrizione | Disponibilità con fromEvent |
|---|---|---|---|---|
| wheel | onwheel | WheelEvent | Quando la rotella del mouse viene ruotata | ✅ |
| abort | onabort | UIEvent | Quando il caricamento delle risorse viene interrotto | ✅ |
| hashchange | onhashchange | HashChangeEvent | Quando l'hash dell'URL (es. `#section1`) viene cambiato | ✅ |
| message | onmessage | MessageEvent | Quando viene ricevuto un messaggio dai Web Worker o da un iframe | ❌ |
| online | ononline | Event | Quando la rete torna online | ✅ |
| offline | onoffline | Event | Quando la rete va offline | ✅ |
| popstate | onpopstate | PopStateEvent | Quando lo stato cambia a causa di `history.pushState` o `history.back` | ❌ |
| storage | onstorage | StorageEvent | Quando `localStorage` o `sessionStorage` viene modificato | ❌ |
| languagechange | onlanguagechange | Event | Quando viene cambiata l'impostazione della lingua (modifica delle impostazioni del browser) | ❌ |
