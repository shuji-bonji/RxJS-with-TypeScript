---
description: "Een overzichtstabel van JavaScript-events die gebruikt kunnen worden met RxJS fromEvent, georganiseerd per categorie. Biedt referentie-informatie per categorie voor het implementeren van event handling, zoals muis-, toetsenbord-, formulier-, touch-, drag & drop-, media- en vensterevents."
---
# Lijst van Events

## 1. Muisevents

| JavaScript Event Naam | HTML Attribuut | Type | Beschrijving | Beschikbaar in fromEvent |
|---|---|---|---|---|
| click | onclick | MouseEvent | Wanneer een element wordt geklikt | ✅ |
| dblclick | ondblclick | MouseEvent | Wanneer een element wordt dubbelgeklikt | ✅ |
| mousedown | onmousedown | MouseEvent | Wanneer een muisknop wordt ingedrukt | ✅ |
| mouseup | onmouseup | MouseEvent | Wanneer een muisknop wordt losgelaten | ✅ |
| mousemove | onmousemove | MouseEvent | Wanneer de muis beweegt | ✅ |
| mouseover | onmouseover | MouseEvent | Wanneer de muis over een element komt | ✅ |
| mouseout | onmouseout | MouseEvent | Wanneer de muis een element verlaat | ✅ |
| mouseenter | onmouseenter | MouseEvent | Wanneer de muis een element binnenkomt (geen bubbling) | ✅ |
| mouseleave | onmouseleave | MouseEvent | Wanneer de muis een element verlaat (geen bubbling) | ✅ |
| contextmenu | oncontextmenu | MouseEvent | Wanneer het rechtermuismenu wordt geopend | ✅ |


## 2. Pointer Events

| JavaScript Event Naam | HTML Attribuut | Type | Beschrijving | Beschikbaar in fromEvent |
|---|---|---|---|---|
| pointerdown | onpointerdown | PointerEvent | Wanneer een pointer (touch, pen, muis) wordt ingedrukt | ✅ |
| pointerup | onpointerup | PointerEvent | Wanneer een pointer wordt losgelaten | ✅ |
| pointermove | onpointermove | PointerEvent | Wanneer een pointer beweegt | ✅ |
| pointerover | onpointerover | PointerEvent | Wanneer een pointer over een element komt | ✅ |
| pointerout | onpointerout | PointerEvent | Wanneer een pointer een element verlaat | ✅ |
| pointerenter | onpointerenter | PointerEvent | Wanneer een pointer een element binnenkomt (geen bubbling) | ✅ |
| pointerleave | onpointerleave | PointerEvent | Wanneer een pointer een element verlaat (geen bubbling) | ✅ |
| pointercancel | onpointercancel | PointerEvent | Wanneer een pointer-operatie wordt geannuleerd | ✅ |
| gotpointercapture | ongotpointercapture | PointerEvent | Wanneer pointer capture wordt verkregen | ✅ |
| lostpointercapture | onlostpointercapture | PointerEvent | Wanneer pointer capture wordt verloren | ✅ |


## 3. Touch Events

| JavaScript Event Naam | HTML Attribuut | Type | Beschrijving | Beschikbaar in fromEvent |
|---|---|---|---|---|
| touchstart | ontouchstart | TouchEvent | Wanneer het scherm wordt aangeraakt | ✅ |
| touchmove | ontouchmove | TouchEvent | Wanneer een aanrakende vinger beweegt | ✅ |
| touchend | ontouchend | TouchEvent | Wanneer een aanraking eindigt | ✅ |
| touchcancel | ontouchcancel | TouchEvent | Wanneer een aanraking wordt geannuleerd | ✅ |


## 4. Toetsenbord Events

| JavaScript Event Naam | HTML Attribuut | Type | Beschrijving | Beschikbaar in fromEvent |
|---|---|---|---|---|
| keydown | onkeydown | KeyboardEvent | Wanneer een toets wordt ingedrukt | ✅ |
| keypress | onkeypress | KeyboardEvent | ⚠️ **Verouderd (Deprecated)** - Gebruik `keydown` | ✅ |
| keyup | onkeyup | KeyboardEvent | Wanneer een toets wordt losgelaten | ✅ |

::: warning Over het keypress event
Het `keypress` event is **verouderd volgens de webstandaard**.

**Redenen voor veroudering**:
- Onvoldoende ondersteuning voor internationalisatie (problemen met Japanse invoer, etc.)
- Instabiel gedrag in combinatie met modifier-toetsen (Shift, Ctrl, Alt)
- Beperkte ondersteuning op mobiele apparaten

**Aanbevolen alternatief**:
```typescript
// ❌ Verouderd
fromEvent(input, 'keypress')
  .subscribe(event => console.log(event));

// ✅ Aanbevolen: gebruik keydown
fromEvent<KeyboardEvent>(input, 'keydown')
  .subscribe(event => console.log(event.key));
```

**Aanbevolen events per use case**:
- Detectie van karakterinvoer: `input` event (aanbevolen)
- Detectie van toetsacties: `keydown` event
- Detectie van toets loslaten: `keyup` event
:::


## 5. Focus-gerelateerde Events

| JavaScript Event Naam | HTML Attribuut | Type | Beschrijving | Beschikbaar in fromEvent |
|---|---|---|---|---|
| focus | onfocus | FocusEvent | Wanneer een element focus krijgt | ✅ |
| blur | onblur | FocusEvent | Wanneer een element focus verliest | ✅ |
| focusin | onfocusin | FocusEvent | Wanneer een element of kindelement focus krijgt | ✅ |
| focusout | onfocusout | FocusEvent | Wanneer een element of kindelement focus verliest | ✅ |


## 6. Formulier Events

| JavaScript Event Naam | HTML Attribuut | Type | Beschrijving | Beschikbaar in fromEvent |
|---|---|---|---|---|
| change | onchange | Event | Wanneer de invoerinhoud is gewijzigd | ✅ |
| input | oninput | InputEvent | Wanneer de waarde van een invoerveld is gewijzigd | ✅ |
| submit | onsubmit | SubmitEvent | Wanneer een formulier wordt verzonden | ✅ |
| reset | onreset | Event | Wanneer een formulier wordt gereset | ✅ |
| select | onselect | Event | Wanneer tekst wordt geselecteerd | ✅ |


## 7. Drag & Drop Events

| JavaScript Event Naam | HTML Attribuut | Type | Beschrijving | Beschikbaar in fromEvent |
|---|---|---|---|---|
| drag | ondrag | DragEvent | Terwijl een element wordt gesleept | ✅ |
| dragstart | ondragstart | DragEvent | Wanneer slepen begint | ✅ |
| dragend | ondragend | DragEvent | Wanneer slepen eindigt | ✅ |
| dragover | ondragover | DragEvent | Wanneer een gesleept element over een ander element hangt | ✅ |
| dragenter | ondragenter | DragEvent | Wanneer een gesleept element een doelelement binnenkomt | ✅ |
| dragleave | ondragleave | DragEvent | Wanneer een gesleept element een doelelement verlaat | ✅ |
| drop | ondrop | DragEvent | Wanneer een gesleept element wordt neergezet | ✅ |


## 8. Venster & Document Events

| JavaScript Event Naam | HTML Attribuut | Type | Beschrijving | Beschikbaar in fromEvent |
|---|---|---|---|---|
| load | onload | Event | Wanneer de pagina volledig is geladen | ✅ |
| resize | onresize | UIEvent | Wanneer de venstergrootte wordt gewijzigd | ✅ |
| scroll | onscroll | Event | Wanneer de pagina wordt gescrolld | ✅ |
| unload | onunload | Event | Wanneer de pagina wordt gesloten | ❌ |
| beforeunload | onbeforeunload | BeforeUnloadEvent | Net voordat de pagina wordt gesloten | ❌ |
| error | onerror | ErrorEvent | Wanneer een fout optreedt | ✅ |
| visibilitychange | onvisibilitychange | Event | Wanneer de zichtbaarheidsstatus van de pagina verandert (zoals tabwisseling) | ✅ |
| fullscreenchange | onfullscreenchange | Event | Wanneer de fullscreen-status verandert | ✅ |


## 9. Media Events

| JavaScript Event Naam | HTML Attribuut | Type | Beschrijving | Beschikbaar in fromEvent |
|---|---|---|---|---|
| play | onplay | Event | Wanneer het afspelen van media begint | ✅ |
| pause | onpause | Event | Wanneer het afspelen van media wordt gepauzeerd | ✅ |
| ended | onended | Event | Wanneer het afspelen van media eindigt | ✅ |
| volumechange | onvolumechange | Event | Wanneer het mediavolume wordt gewijzigd | ✅ |
| seeking | onseeking | Event | Wanneer media zoeken begint | ✅ |
| seeked | onseeked | Event | Wanneer media zoeken is voltooid | ✅ |
| timeupdate | ontimeupdate | Event | Wanneer de afspeeltijd van media wordt bijgewerkt | ✅ |


## 10. Animatie & Transitie Events

| JavaScript Event Naam | HTML Attribuut | Type | Beschrijving | Beschikbaar in fromEvent |
|---|---|---|---|---|
| animationstart | onanimationstart | AnimationEvent | Wanneer een animatie begint | ✅ |
| animationend | onanimationend | AnimationEvent | Wanneer een animatie eindigt | ✅ |
| animationiteration | onanimationiteration | AnimationEvent | Wanneer een animatie wordt herhaald | ✅ |
| transitionstart | ontransitionstart | TransitionEvent | Wanneer een CSS-transitie begint | ✅ |
| transitionend | ontransitionend | TransitionEvent | Wanneer een CSS-transitie eindigt | ✅ |


## 11. Overige Events

| JavaScript Event Naam | HTML Attribuut | Type | Beschrijving | Beschikbaar in fromEvent |
|---|---|---|---|---|
| wheel | onwheel | WheelEvent | Wanneer het muiswiel wordt gedraaid | ✅ |
| abort | onabort | UIEvent | Wanneer het laden van een resource wordt afgebroken | ✅ |
| hashchange | onhashchange | HashChangeEvent | Wanneer de URL-hash (zoals `#section1`) wordt gewijzigd | ✅ |
| message | onmessage | MessageEvent | Wanneer een bericht wordt ontvangen van Web Workers of iframe | ❌ |
| online | ononline | Event | Wanneer het netwerk weer online komt | ✅ |
| offline | onoffline | Event | Wanneer het netwerk offline gaat | ✅ |
| popstate | onpopstate | PopStateEvent | Bij statuswijzigingen door `history.pushState` of `history.back` | ❌ |
| storage | onstorage | StorageEvent | Wanneer `localStorage` of `sessionStorage` wordt gewijzigd | ❌ |
| languagechange | onlanguagechange | Event | Wanneer de taalinstelling wordt gewijzigd (browserinstellingen) | ❌ |
