---
description: "Complete list of JavaScript events for RxJS fromEvent: Mouse, pointer, touch, keyboard, form, drag-and-drop, media, and animation events organized by category"
---
# List of Events

## 1. Mouse Events

| JavaScript Event Name | HTML Attribute | Type | Description | Available in fromEvent |
|---|---|---|---|---|
| click | onclick | MouseEvent | When an element is clicked | ✅ |
| dblclick | ondblclick | MouseEvent | When an element is double-clicked | ✅ |
| mousedown | onmousedown | MouseEvent | When the mouse button is pressed | ✅ |
| mouseup | onmouseup | MouseEvent | When the mouse button is released | ✅ |
| mousemove | onmousemove | MouseEvent | When the mouse is moved | ✅ |
| mouseover | onmouseover | MouseEvent | When the mouse is over an element | ✅ |
| mouseout | onmouseout | MouseEvent | When the mouse moves out of the element | ✅ |
| mouseenter | onmouseenter | MouseEvent | When the mouse enters an element (without bubbling) | ✅ |
| mouseleave | onmouseleave | MouseEvent | When the mouse leaves the element (no bubbling) | ✅ |
| contextmenu | oncontextmenu | MouseEvent | When the right-click menu is opened | ✅ |

## 2. Pointer Events

| JavaScript Event Name | HTML Attribute | Type | Description | Available in fromEvent |
|---|---|---|---|---|
| pointerdown | onpointerdown | PointerEvent | When the pointer (touch, pen, mouse) is pressed | ✅ |
| pointerup | onpointerup | PointerEvent | When the pointer is released | ✅ |
| pointermove | onpointermove | PointerEvent | When the pointer is moved | ✅ |
| pointerover | onpointerover | PointerEvent | When the pointer is on an element | ✅ |
| pointerout | onpointerout | PointerEvent | When the pointer moves out of the element | ✅ |
| pointerenter | onpointerenter | PointerEvent | When the pointer enters an element (without bubbling) | ✅ |
| pointerleave | onpointerleave | PointerEvent | When the pointer leaves the element (no bubbling) | ✅ |
| pointercancel | onpointercancel | PointerEvent | When the pointer operation is canceled | ✅ |
| gotpointercapture | ongotpointercapture | PointerEvent | When the pointer capture is acquired | ✅ |
| lostpointercapture | onlostpointercapture | PointerEvent | When the pointer capture is lost | ✅ |

## 3. Touch Events

| JavaScript Event Name | HTML Attribute | Type | Description | Available in fromEvent |
|---|---|---|---|---|
| touchstart | ontouchstart | TouchEvent | When the screen is touched | ✅ |
| touchmove | ontouchmove | TouchEvent | When the touched finger moves | ✅ |
| touchend | ontouchend | TouchEvent | When a touch is terminated | ✅ |
| touchcancel | ontouchcancel | TouchEvent | When a touch is canceled | ✅ |

## 4. Keyboard Events

| JavaScript Event Name | HTML Attribute | Type | Description | Available in fromEvent |
|---|---|---|---|---|
| keydown | onkeydown | KeyboardEvent | When a key is pressed | ✅ |
| keypress | onkeypress | KeyboardEvent | ⚠️ **Deprecated** - Use `keydown` instead | ✅ |
| keyup | onkeyup | KeyboardEvent | When a key is released | ✅ |

::: warning About the keypress Event
The `keypress` event has been **deprecated** by web standards.

**Reasons for deprecation**:
- Insufficient internationalization support (problems with Japanese input, etc.)
- Unstable behavior in combination with modifier keys (Shift, Ctrl, Alt)
- Limited support for mobile devices

**Recommended alternatives**:
```typescript
// ❌ Deprecated
fromEvent(input, 'keypress')
  .subscribe(event => console.log(event));

// ✅ Recommended: Use keydown
fromEvent<KeyboardEvent>(input, 'keydown')
  .subscribe(event => console.log(event.key));
```

**Recommended events by use case**:
- Detection of text input: `input` event (recommended)
- Detection of keystroke: `keydown` event
- Detection of key release: `keyup` event
:::

## 5. Focus-related Events

| JavaScript Event Name | HTML Attribute | Type | Description | Available in fromEvent |
|---|---|---|---|---|
| focus | onfocus | FocusEvent | When an element receives focus | ✅ |
| blur | onblur | FocusEvent | When an element loses focus | ✅ |
| focusin | onfocusin | FocusEvent | When an element or child element receives focus | ✅ |
| focusout | onfocusout | FocusEvent | When an element or child element loses focus | ✅ |

## 6. Form Events

| JavaScript Event Name | HTML Attribute | Type | Description | Available in fromEvent |
|---|---|---|---|---|
| change | onchange | Event | When the input content is changed | ✅ |
| input | oninput | InputEvent | When the value of an input field is changed | ✅ |
| submit | onsubmit | SubmitEvent | When the form is submitted | ✅ |
| reset | onreset | Event | When the form is reset | ✅ |
| select | onselect | Event | When text is selected | ✅ |

## 7. Drag & Drop Events

| JavaScript Event Name | HTML Attribute | Type | Description | Available in fromEvent |
|---|---|---|---|---|
| drag | ondrag | DragEvent | While the element is being dragged | ✅ |
| dragstart | ondragstart | DragEvent | When a drag is started | ✅ |
| dragend | ondragend | DragEvent | When the dragging ends | ✅ |
| dragover | ondragover | DragEvent | When the dragged element is on top of another element | ✅ |
| dragenter | ondragenter | DragEvent | When the dragged element enters the target | ✅ |
| dragleave | ondragleave | DragEvent | When the dragged element is off the target | ✅ |
| drop | ondrop | DragEvent | When the dragged element is dropped | ✅ |

## 8. Window & Document Events

| JavaScript Event Name | HTML Attribute | Type | Description | Available in fromEvent |
|---|---|---|---|---|
| load | onload | Event | When the page is completely loaded | ✅ |
| resize | onresize | UIEvent | When the window is resized | ✅ |
| scroll | onscroll | Event | When a page is scrolled | ✅ |
| unload | onunload | Event | When the page is closed | ❌ |
| beforeunload | onbeforeunload | BeforeUnloadEvent | Just before the page is closed | ❌ |
| error | onerror | ErrorEvent | When an error occurs | ✅ |
| visibilitychange | onvisibilitychange | Event | When the page display state changes (e.g., switching tabs) | ✅ |
| fullscreenchange | onfullscreenchange | Event | When the full-screen status changes | ✅ |

## 9. Media Events

| JavaScript Event Name | HTML Attribute | Type | Description | Available in fromEvent |
|---|---|---|---|---|
| play | onplay | Event | When media playback starts | ✅ |
| pause | onpause | Event | When media playback is paused | ✅ |
| ended | onended | Event | When media playback ends | ✅ |
| volumechange | onvolumechange | Event | When media volume is changed | ✅ |
| seeking | onseeking | Event | When seeking of media is started | ✅ |
| seeked | onseeked | Event | When media seek is completed | ✅ |
| timeupdate | ontimeupdate | Event | When the media playback time is updated | ✅ |

## 10. Animation & Transition Events

| JavaScript Event Name | HTML Attribute | Type | Description | Available in fromEvent |
|---|---|---|---|---|
| animationstart | onanimationstart | AnimationEvent | When an animation starts | ✅ |
| animationend | onanimationend | AnimationEvent | When the animation ends | ✅ |
| animationiteration | onanimationiteration | AnimationEvent | When the animation is repeated | ✅ |
| transitionstart | ontransitionstart | TransitionEvent | When a CSS transition starts | ✅ |
| transitionend | ontransitionend | TransitionEvent | When a CSS transition ends | ✅ |

## 11. Other Events

| JavaScript Event Name | HTML Attribute | Type | Description | Available in fromEvent |
|---|---|---|---|---|
| wheel | onwheel | WheelEvent | When the mouse wheel is rotated | ✅ |
| abort | onabort | UIEvent | When resource loading is interrupted | ✅ |
| hashchange | onhashchange | HashChangeEvent | When the URL hash (e.g. `#section1`) is changed | ✅ |
| message | onmessage | MessageEvent | When a message is received from Web Workers or iframes | ❌ |
| online | ononline | Event | When the network comes back online | ✅ |
| offline | onoffline | Event | When the network goes offline | ✅ |
| popstate | onpopstate | PopStateEvent | When a state change occurs due to `history.pushState` or `history.back` | ❌ |
| storage | onstorage | StorageEvent | When `localStorage` or `sessionStorage` is changed | ❌ |
| languagechange | onlanguagechange | Event | When the language setting is changed (browser setting change) | ❌ |
