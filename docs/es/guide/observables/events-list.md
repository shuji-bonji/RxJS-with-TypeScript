---
description: "Lista completa de eventos JavaScript para fromEvent de RxJS: Mouse, puntero, táctil, teclado, formulario, arrastrar y soltar, medios y eventos de animación organizados por categoría"
---
# Lista de Eventos

## 1. Eventos de Mouse

| Nombre del Evento JavaScript | Atributo HTML | Tipo | Descripción | Disponible en fromEvent |
|---|---|---|---|---|
| click | onclick | MouseEvent | Cuando se hace clic en un elemento | ✅ |
| dblclick | ondblclick | MouseEvent | Cuando se hace doble clic en un elemento | ✅ |
| mousedown | onmousedown | MouseEvent | Cuando se presiona el botón del mouse | ✅ |
| mouseup | onmouseup | MouseEvent | Cuando se suelta el botón del mouse | ✅ |
| mousemove | onmousemove | MouseEvent | Cuando se mueve el mouse | ✅ |
| mouseover | onmouseover | MouseEvent | Cuando el mouse está sobre un elemento | ✅ |
| mouseout | onmouseout | MouseEvent | Cuando el mouse sale del elemento | ✅ |
| mouseenter | onmouseenter | MouseEvent | Cuando el mouse entra en un elemento (sin burbujeo) | ✅ |
| mouseleave | onmouseleave | MouseEvent | Cuando el mouse sale del elemento (sin burbujeo) | ✅ |
| contextmenu | oncontextmenu | MouseEvent | Cuando se abre el menú de clic derecho | ✅ |

## 2. Eventos de Puntero

| Nombre del Evento JavaScript | Atributo HTML | Tipo | Descripción | Disponible en fromEvent |
|---|---|---|---|---|
| pointerdown | onpointerdown | PointerEvent | Cuando se presiona el puntero (táctil, pluma, mouse) | ✅ |
| pointerup | onpointerup | PointerEvent | Cuando se suelta el puntero | ✅ |
| pointermove | onpointermove | PointerEvent | Cuando se mueve el puntero | ✅ |
| pointerover | onpointerover | PointerEvent | Cuando el puntero está sobre un elemento | ✅ |
| pointerout | onpointerout | PointerEvent | Cuando el puntero sale del elemento | ✅ |
| pointerenter | onpointerenter | PointerEvent | Cuando el puntero entra en un elemento (sin burbujeo) | ✅ |
| pointerleave | onpointerleave | PointerEvent | Cuando el puntero sale del elemento (sin burbujeo) | ✅ |
| pointercancel | onpointercancel | PointerEvent | Cuando se cancela la operación del puntero | ✅ |
| gotpointercapture | ongotpointercapture | PointerEvent | Cuando se adquiere la captura del puntero | ✅ |
| lostpointercapture | onlostpointercapture | PointerEvent | Cuando se pierde la captura del puntero | ✅ |

## 3. Eventos Táctiles

| Nombre del Evento JavaScript | Atributo HTML | Tipo | Descripción | Disponible en fromEvent |
|---|---|---|---|---|
| touchstart | ontouchstart | TouchEvent | Cuando se toca la pantalla | ✅ |
| touchmove | ontouchmove | TouchEvent | Cuando se mueve el dedo tocado | ✅ |
| touchend | ontouchend | TouchEvent | Cuando termina un toque | ✅ |
| touchcancel | ontouchcancel | TouchEvent | Cuando se cancela un toque | ✅ |

## 4. Eventos de Teclado

| Nombre del Evento JavaScript | Atributo HTML | Tipo | Descripción | Disponible en fromEvent |
|---|---|---|---|---|
| keydown | onkeydown | KeyboardEvent | Cuando se presiona una tecla | ✅ |
| keypress | onkeypress | KeyboardEvent | ⚠️ **Obsoleto** - Use `keydown` en su lugar | ✅ |
| keyup | onkeyup | KeyboardEvent | Cuando se suelta una tecla | ✅ |

::: warning Sobre el evento keypress
El evento `keypress` ha sido **obsoleto** por los estándares web.

**Razones para la obsolescencia**:
- Soporte de internacionalización insuficiente (problemas con entrada japonesa, etc.)
- Comportamiento inestable en combinación con teclas modificadoras (Shift, Ctrl, Alt)
- Soporte limitado para dispositivos móviles

**Alternativas recomendadas**:
```typescript
// ❌ Obsoleto
fromEvent(input, 'keypress')
  .subscribe(event => console.log(event));

// ✅ Recomendado: Use keydown
fromEvent<KeyboardEvent>(input, 'keydown')
  .subscribe(event => console.log(event.key));
```

**Eventos recomendados por caso de uso**:
- Detección de entrada de texto: evento `input` (recomendado)
- Detección de pulsación de tecla: evento `keydown`
- Detección de liberación de tecla: evento `keyup`
:::

## 5. Eventos Relacionados con Enfoque

| Nombre del Evento JavaScript | Atributo HTML | Tipo | Descripción | Disponible en fromEvent |
|---|---|---|---|---|
| focus | onfocus | FocusEvent | Cuando un elemento recibe foco | ✅ |
| blur | onblur | FocusEvent | Cuando un elemento pierde foco | ✅ |
| focusin | onfocusin | FocusEvent | Cuando un elemento o elemento hijo recibe foco | ✅ |
| focusout | onfocusout | FocusEvent | Cuando un elemento o elemento hijo pierde foco | ✅ |

## 6. Eventos de Formulario

| Nombre del Evento JavaScript | Atributo HTML | Tipo | Descripción | Disponible en fromEvent |
|---|---|---|---|---|
| change | onchange | Event | Cuando se cambia el contenido de entrada | ✅ |
| input | oninput | InputEvent | Cuando se cambia el valor de un campo de entrada | ✅ |
| submit | onsubmit | SubmitEvent | Cuando se envía el formulario | ✅ |
| reset | onreset | Event | Cuando se reinicia el formulario | ✅ |
| select | onselect | Event | Cuando se selecciona texto | ✅ |

## 7. Eventos de Arrastrar y Soltar

| Nombre del Evento JavaScript | Atributo HTML | Tipo | Descripción | Disponible en fromEvent |
|---|---|---|---|---|
| drag | ondrag | DragEvent | Mientras se arrastra el elemento | ✅ |
| dragstart | ondragstart | DragEvent | Cuando se inicia un arrastre | ✅ |
| dragend | ondragend | DragEvent | Cuando termina el arrastre | ✅ |
| dragover | ondragover | DragEvent | Cuando el elemento arrastrado está sobre otro elemento | ✅ |
| dragenter | ondragenter | DragEvent | Cuando el elemento arrastrado entra en el objetivo | ✅ |
| dragleave | ondragleave | DragEvent | Cuando el elemento arrastrado sale del objetivo | ✅ |
| drop | ondrop | DragEvent | Cuando se suelta el elemento arrastrado | ✅ |

## 8. Eventos de Ventana y Documento

| Nombre del Evento JavaScript | Atributo HTML | Tipo | Descripción | Disponible en fromEvent |
|---|---|---|---|---|
| load | onload | Event | Cuando la página está completamente cargada | ✅ |
| resize | onresize | UIEvent | Cuando se redimensiona la ventana | ✅ |
| scroll | onscroll | Event | Cuando se desplaza una página | ✅ |
| unload | onunload | Event | Cuando se cierra la página | ❌ |
| beforeunload | onbeforeunload | BeforeUnloadEvent | Justo antes de cerrar la página | ❌ |
| error | onerror | ErrorEvent | Cuando ocurre un error | ✅ |
| visibilitychange | onvisibilitychange | Event | Cuando cambia el estado de visualización de la página (ej., cambio de pestañas) | ✅ |
| fullscreenchange | onfullscreenchange | Event | Cuando cambia el estado de pantalla completa | ✅ |

## 9. Eventos de Medios

| Nombre del Evento JavaScript | Atributo HTML | Tipo | Descripción | Disponible en fromEvent |
|---|---|---|---|---|
| play | onplay | Event | Cuando comienza la reproducción de medios | ✅ |
| pause | onpause | Event | Cuando se pausa la reproducción de medios | ✅ |
| ended | onended | Event | Cuando termina la reproducción de medios | ✅ |
| volumechange | onvolumechange | Event | Cuando se cambia el volumen de medios | ✅ |
| seeking | onseeking | Event | Cuando se inicia la búsqueda de medios | ✅ |
| seeked | onseeked | Event | Cuando se completa la búsqueda de medios | ✅ |
| timeupdate | ontimeupdate | Event | Cuando se actualiza el tiempo de reproducción de medios | ✅ |

## 10. Eventos de Animación y Transición

| Nombre del Evento JavaScript | Atributo HTML | Tipo | Descripción | Disponible en fromEvent |
|---|---|---|---|---|
| animationstart | onanimationstart | AnimationEvent | Cuando comienza una animación | ✅ |
| animationend | onanimationend | AnimationEvent | Cuando termina la animación | ✅ |
| animationiteration | onanimationiteration | AnimationEvent | Cuando se repite la animación | ✅ |
| transitionstart | ontransitionstart | TransitionEvent | Cuando comienza una transición CSS | ✅ |
| transitionend | ontransitionend | TransitionEvent | Cuando termina una transición CSS | ✅ |

## 11. Otros Eventos

| Nombre del Evento JavaScript | Atributo HTML | Tipo | Descripción | Disponible en fromEvent |
|---|---|---|---|---|
| wheel | onwheel | WheelEvent | Cuando se gira la rueda del mouse | ✅ |
| abort | onabort | UIEvent | Cuando se interrumpe la carga de recursos | ✅ |
| hashchange | onhashchange | HashChangeEvent | Cuando se cambia el hash de URL (ej. `#section1`) | ✅ |
| message | onmessage | MessageEvent | Cuando se recibe un mensaje de Web Workers o iframes | ❌ |
| online | ononline | Event | Cuando la red vuelve a estar en línea | ✅ |
| offline | onoffline | Event | Cuando la red se desconecta | ✅ |
| popstate | onpopstate | PopStateEvent | Cuando ocurre un cambio de estado debido a `history.pushState` o `history.back` | ❌ |
| storage | onstorage | StorageEvent | Cuando se cambia `localStorage` o `sessionStorage` | ❌ |
| languagechange | onlanguagechange | Event | Cuando se cambia la configuración de idioma (cambio de configuración del navegador) | ❌ |
