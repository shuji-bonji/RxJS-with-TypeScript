---
description: "Übersicht über in RxJS fromEvent verwendbare JavaScript-Events, nach Typ organisiert. Bereitstellung von Referenzinformationen nach Kategorien für Event-Verarbeitungsimplementierung wie Maus, Tastatur, Formulare, Touch, Drag & Drop, Medien, Fenster usw."
---
# Event-Übersichtstabelle

## 1. Maus-Events

| JavaScript Event-Name | HTML-Attribut | Typ | Beschreibung | fromEvent verfügbar |
|---|---|---|---|---|
| click | onclick | MouseEvent | Wenn Element geklickt wird | ✅ |
| dblclick | ondblclick | MouseEvent | Wenn Element doppelt geklickt wird | ✅ |
| mousedown | onmousedown | MouseEvent | Wenn Maustaste gedrückt wird | ✅ |
| mouseup | onmouseup | MouseEvent | Wenn Maustaste losgelassen wird | ✅ |
| mousemove | onmousemove | MouseEvent | Wenn Maus bewegt wird | ✅ |
| mouseover | onmouseover | MouseEvent | Wenn Maus über Element kommt | ✅ |
| mouseout | onmouseout | MouseEvent | Wenn Maus Element verlässt | ✅ |
| mouseenter | onmouseenter | MouseEvent | Wenn Maus Element betritt (kein Bubbling) | ✅ |
| mouseleave | onmouseleave | MouseEvent | Wenn Maus Element verlässt (kein Bubbling) | ✅ |
| contextmenu | oncontextmenu | MouseEvent | Wenn Rechtsklick-Menü geöffnet wird | ✅ |


## 2. Pointer-Events

| JavaScript Event-Name | HTML-Attribut | Typ | Beschreibung | fromEvent verfügbar |
|---|---|---|---|---|
| pointerdown | onpointerdown | PointerEvent | Wenn Pointer (Touch, Stift, Maus) gedrückt wird | ✅ |
| pointerup | onpointerup | PointerEvent | Wenn Pointer losgelassen wird | ✅ |
| pointermove | onpointermove | PointerEvent | Wenn Pointer bewegt wird | ✅ |
| pointerover | onpointerover | PointerEvent | Wenn Pointer über Element kommt | ✅ |
| pointerout | onpointerout | PointerEvent | Wenn Pointer Element verlässt | ✅ |
| pointerenter | onpointerenter | PointerEvent | Wenn Pointer Element betritt (kein Bubbling) | ✅ |
| pointerleave | onpointerleave | PointerEvent | Wenn Pointer Element verlässt (kein Bubbling) | ✅ |
| pointercancel | onpointercancel | PointerEvent | Wenn Pointer-Operation abgebrochen wird | ✅ |
| gotpointercapture | ongotpointercapture | PointerEvent | Wenn Pointer-Capture erhalten wird | ✅ |
| lostpointercapture | onlostpointercapture | PointerEvent | Wenn Pointer-Capture verloren geht | ✅ |


## 3. Touch-Events

| JavaScript Event-Name | HTML-Attribut | Typ | Beschreibung | fromEvent verfügbar |
|---|---|---|---|---|
| touchstart | ontouchstart | TouchEvent | Wenn Bildschirm berührt wird | ✅ |
| touchmove | ontouchmove | TouchEvent | Wenn berührender Finger bewegt wird | ✅ |
| touchend | ontouchend | TouchEvent | Wenn Berührung endet | ✅ |
| touchcancel | ontouchcancel | TouchEvent | Wenn Berührung abgebrochen wird | ✅ |


## 4. Tastatur-Events

| JavaScript Event-Name | HTML-Attribut | Typ | Beschreibung | fromEvent verfügbar |
|---|---|---|---|---|
| keydown | onkeydown | KeyboardEvent | Wenn Taste gedrückt wird | ✅ |
| keypress | onkeypress | KeyboardEvent | ⚠️ **Veraltet (Deprecated)** - Verwenden Sie `keydown` | ✅ |
| keyup | onkeyup | KeyboardEvent | Wenn Taste losgelassen wird | ✅ |

::: warning Über keypress Event
Das `keypress`-Event ist **im Webstandard als veraltet** markiert.

**Gründe für Veraltung**:
- Unzureichende Internationalisierungsunterstützung (Probleme bei japanischer Eingabe etc.)
- Instabiles Verhalten in Kombination mit Modifikatortasten (Shift, Ctrl, Alt)
- Eingeschränkte Unterstützung auf mobilen Geräten

**Empfohlene Alternative**:
```typescript
// ❌ Veraltet
fromEvent(input, 'keypress')
  .subscribe(event => console.log(event));

// ✅ Empfohlen: Verwenden Sie keydown
fromEvent<KeyboardEvent>(input, 'keydown')
  .subscribe(event => console.log(event.key));
```

**Empfohlene Events nach Anwendungsfall**:
- Zeicheneingabe erkennen: `input`-Event (empfohlen)
- Tastenoperation erkennen: `keydown`-Event
- Tastenfreigabe erkennen: `keyup`-Event
:::


## 5. Fokus-bezogene Events

| JavaScript Event-Name | HTML-Attribut | Typ | Beschreibung | fromEvent verfügbar |
|---|---|---|---|---|
| focus | onfocus | FocusEvent | Wenn Element Fokus erhält | ✅ |
| blur | onblur | FocusEvent | Wenn Element Fokus verliert | ✅ |
| focusin | onfocusin | FocusEvent | Wenn Element oder Kindelement Fokus erhält | ✅ |
| focusout | onfocusout | FocusEvent | Wenn Element oder Kindelement Fokus verliert | ✅ |


## 6. Formular-Events

| JavaScript Event-Name | HTML-Attribut | Typ | Beschreibung | fromEvent verfügbar |
|---|---|---|---|---|
| change | onchange | Event | Wenn Eingabeinhalt geändert wird | ✅ |
| input | oninput | InputEvent | Wenn Wert des Eingabefelds geändert wird | ✅ |
| submit | onsubmit | SubmitEvent | Wenn Formular abgesendet wird | ✅ |
| reset | onreset | Event | Wenn Formular zurückgesetzt wird | ✅ |
| select | onselect | Event | Wenn Text ausgewählt wird | ✅ |


## 7. Drag & Drop-Events

| JavaScript Event-Name | HTML-Attribut | Typ | Beschreibung | fromEvent verfügbar |
|---|---|---|---|---|
| drag | ondrag | DragEvent | Während Element gezogen wird | ✅ |
| dragstart | ondragstart | DragEvent | Wenn Ziehen beginnt | ✅ |
| dragend | ondragend | DragEvent | Wenn Ziehen endet | ✅ |
| dragover | ondragover | DragEvent | Wenn gezogenes Element über anderem Element ist | ✅ |
| dragenter | ondragenter | DragEvent | Wenn gezogenes Element Ziel betritt | ✅ |
| dragleave | ondragleave | DragEvent | Wenn gezogenes Element Ziel verlässt | ✅ |
| drop | ondrop | DragEvent | Wenn gezogenes Element abgelegt wird | ✅ |


## 8. Fenster & Dokument-Events

| JavaScript Event-Name | HTML-Attribut | Typ | Beschreibung | fromEvent verfügbar |
|---|---|---|---|---|
| load | onload | Event | Wenn Seite vollständig geladen ist | ✅ |
| resize | onresize | UIEvent | Wenn Fenstergröße geändert wird | ✅ |
| scroll | onscroll | Event | Wenn Seite gescrollt wird | ✅ |
| unload | onunload | Event | Wenn Seite geschlossen wird | ❌ |
| beforeunload | onbeforeunload | BeforeUnloadEvent | Unmittelbar bevor Seite geschlossen wird | ❌ |
| error | onerror | ErrorEvent | Wenn Fehler auftritt | ✅ |
| visibilitychange | onvisibilitychange | Event | Wenn Sichtbarkeitsstatus der Seite sich ändert (Tab-Wechsel etc.) | ✅ |
| fullscreenchange | onfullscreenchange | Event | Wenn Vollbild-Status sich ändert | ✅ |


## 9. Medien-Events

| JavaScript Event-Name | HTML-Attribut | Typ | Beschreibung | fromEvent verfügbar |
|---|---|---|---|---|
| play | onplay | Event | Wenn Medienwiedergabe startet | ✅ |
| pause | onpause | Event | Wenn Medienwiedergabe pausiert wird | ✅ |
| ended | onended | Event | Wenn Medienwiedergabe endet | ✅ |
| volumechange | onvolumechange | Event | Wenn Medienlautstärke geändert wird | ✅ |
| seeking | onseeking | Event | Wenn Medien-Seek startet | ✅ |
| seeked | onseeked | Event | Wenn Medien-Seek abgeschlossen ist | ✅ |
| timeupdate | ontimeupdate | Event | Wenn Medien-Wiedergabezeit aktualisiert wird | ✅ |


## 10. Animations- & Transitions-Events

| JavaScript Event-Name | HTML-Attribut | Typ | Beschreibung | fromEvent verfügbar |
|---|---|---|---|---|
| animationstart | onanimationstart | AnimationEvent | Wenn Animation startet | ✅ |
| animationend | onanimationend | AnimationEvent | Wenn Animation endet | ✅ |
| animationiteration | onanimationiteration | AnimationEvent | Wenn Animation wiederholt wird | ✅ |
| transitionstart | ontransitionstart | TransitionEvent | Wenn CSS-Transition startet | ✅ |
| transitionend | ontransitionend | TransitionEvent | Wenn CSS-Transition endet | ✅ |


## 11. Sonstige Events

| JavaScript Event-Name | HTML-Attribut | Typ | Beschreibung | fromEvent verfügbar |
|---|---|---|---|---|
| wheel | onwheel | WheelEvent | Wenn Mausrad gedreht wird | ✅ |
| abort | onabort | UIEvent | Wenn Laden der Ressource abgebrochen wird | ✅ |
| hashchange | onhashchange | HashChangeEvent | Wenn URL-Hash (`#section1` etc.) geändert wird | ✅ |
| message | onmessage | MessageEvent | Wenn Nachricht von Web Workers oder iframe empfangen wird | ❌ |
| online | ononline | Event | Wenn Netzwerk wieder online wird | ✅ |
| offline | onoffline | Event | Wenn Netzwerk offline wird | ✅ |
| popstate | onpopstate | PopStateEvent | Bei Statusänderung durch `history.pushState` oder `history.back` | ❌ |
| storage | onstorage | StorageEvent | Wenn `localStorage` oder `sessionStorage` geändert wird | ❌ |
| languagechange | onlanguagechange | Event | Wenn Spracheinstellung geändert wird (Browser-Einstellungsänderung) | ❌ |
