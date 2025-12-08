---
description: "Liste complète des événements JavaScript pour RxJS fromEvent : Événements liés à la souris, au pointeur, au toucher, au clavier, au formulaire, au glisser-déposer, aux médias et à l'animation, organisés par catégorie"
---
# Liste des événements

## 1. Événements de la souris

| Nom de l'événement JavaScript | Attribut HTML | Type | Description | Disponible dans fromEvent |
|---|---|---|---|---|
| click | onclick | MouseEvent | Lorsqu'un élément est cliqué | ✅ |
| dblclick | ondblclick | MouseEvent | Lorsqu'un élément est double-cliqué | ✅ |
| mousedown | onmousedown | MouseEvent | Lorsque le bouton de la souris est enfoncé | ✅ |
| mouseup | onmouseup | MouseEvent | Lorsque le bouton de la souris est relâché | ✅ |
| mousemove | onmousemove | MouseEvent | Lorsque la souris est déplacée | ✅ |
| mouseover | onmouseover | MouseEvent | Lorsque la souris est au-dessus d'un élément | ✅ |
| mouseout | onmouseout | MouseEvent | Lorsque la souris sort de l'élément | ✅ |
| mouseenter | onmouseenter | MouseEvent | Lorsque la souris entre dans un élément (sans propagation) | ✅ |
| mouseleave | onmouseleave | MouseEvent | Lorsque la souris quitte l'élément (sans propagation) | ✅ |
| contextmenu | oncontextmenu | MouseEvent | Lorsque le menu contextuel est ouvert | ✅ |

## 2. Événements liés au pointeur

| Nom de l'événement JavaScript | Attribut HTML | Type | Description | Disponible dans fromEvent |
|---|---|---|---|---|
| pointerdown | onpointerdown | PointerEvent | Lorsque le pointeur (tactile, stylo, souris) est pressé | ✅ |
| pointerup | onpointerup | PointerEvent | Lorsque le pointeur est relâché | ✅ |
| pointermove | onpointermove | PointerEvent | Lorsque le pointeur est déplacé | ✅ |
| pointerover | onpointerover | PointerEvent | Lorsque le pointeur est sur un élément | ✅ |
| pointerout | onpointerout | PointerEvent | Lorsque le pointeur sort de l'élément | ✅ |
| pointerenter | onpointerenter | PointerEvent | Lorsque le pointeur entre dans un élément (sans propagation) | ✅ |
| pointerleave | onpointerleave | PointerEvent | Lorsque le pointeur quitte l'élément (sans propagation) | ✅ |
| pointercancel | onpointercancel | PointerEvent | Lorsque l'opération du pointeur est annulée | ✅ |
| gotpointercapture | ongotpointercapture | PointerEvent | Lorsque la capture du pointeur est acquise | ✅ |
| lostpointercapture | onlostpointercapture | PointerEvent | Lorsque la capture du pointeur est perdue | ✅ |

## 3. Événements tactiles

| Nom de l'événement JavaScript | Attribut HTML | Type | Description | Disponible dans fromEvent |
|---|---|---|---|---|
| touchstart | ontouchstart | TouchEvent | Lorsque l'écran est touché | ✅ |
| touchmove | ontouchmove | TouchEvent | Lorsque le doigt touché se déplace | ✅ |
| touchend | ontouchend | TouchEvent | Lorsqu'un toucher est terminé | ✅ |
| touchcancel | ontouchcancel | TouchEvent | Lorsqu'un toucher est annulé | ✅ |

## 4. Événements clavier

| Nom de l'événement JavaScript | Attribut HTML | Type | Description | Disponible dans fromEvent |
|---|---|---|---|---|
| keydown | onkeydown | KeyboardEvent | Lorsqu'une touche est enfoncée | ✅ |
| keypress | onkeypress | KeyboardEvent | ⚠️ **Déprécié** - Utilisez `keydown` à la place | ✅ |
| keyup | onkeyup | KeyboardEvent | Lorsqu'une touche est relâchée | ✅ |

::: warning À propos de l'événement keypress
L'événement `keypress` a été **déprécié** par les standards du web.

**Raisons de la dépréciation** :
- Prise en charge insuffisante de l'internationalisation (problèmes avec la saisie en japonais, etc.)
- Comportement instable en combinaison avec les touches de modification (Shift, Ctrl, Alt)
- Prise en charge limitée des appareils mobiles

**Alternatives recommandées** :
```typescript
// ❌ Déprécié
fromEvent(input, 'keypress')
  .subscribe(event => console.log(event));

// ✅ Recommandé : Utilisez keydown
fromEvent<KeyboardEvent>(input, 'keydown')
  .subscribe(event => console.log(event.key));
```

**Événements recommandés par cas d'utilisation** :
- Détection de saisie de texte : événement `input` (recommandé)
- Détection de frappe de touche : événement `keydown`
- Détection de relâchement de touche : événement `keyup`
:::

## 5. Événements liés au focus

| Nom de l'événement JavaScript | Attribut HTML | Type | Description | Disponible dans fromEvent |
|---|---|---|---|---|
| focus | onfocus | FocusEvent | Lorsqu'un élément reçoit le focus | ✅ |
| blur | onblur | FocusEvent | Lorsqu'un élément perd le focus | ✅ |
| focusin | onfocusin | FocusEvent | Lorsqu'un élément ou un élément enfant reçoit le focus | ✅ |
| focusout | onfocusout | FocusEvent | Lorsqu'un élément ou un élément enfant perd le focus | ✅ |

## 6. Événements de formulaire

| Nom de l'événement JavaScript | Attribut HTML | Type | Description | Disponible dans fromEvent |
|---|---|---|---|---|
| change | onchange | Event | Lorsque le contenu de l'entrée est modifié | ✅ |
| input | oninput | InputEvent | Lorsque la valeur d'un champ de saisie est modifiée | ✅ |
| submit | onsubmit | SubmitEvent | Lorsque le formulaire est soumis | ✅ |
| reset | onreset | Event | Lorsque le formulaire est réinitialisé | ✅ |
| select | onselect | Event | Lorsque du texte est sélectionné | ✅ |

## 7. Événements de glisser-déposer

| Nom de l'événement JavaScript | Attribut HTML | Type | Description | Disponible dans fromEvent |
|---|---|---|---|---|
| drag | ondrag | DragEvent | Pendant que l'élément est glissé | ✅ |
| dragstart | ondragstart | DragEvent | Lorsqu'un glissement commence | ✅ |
| dragend | ondragend | DragEvent | Lorsque le glissement se termine | ✅ |
| dragover | ondragover | DragEvent | Lorsque l'élément glissé est au-dessus d'un autre élément | ✅ |
| dragenter | ondragenter | DragEvent | Lorsque l'élément glissé entre dans la cible | ✅ |
| dragleave | ondragleave | DragEvent | Lorsque l'élément glissé quitte la cible | ✅ |
| drop | ondrop | DragEvent | Lorsque l'élément glissé est déposé | ✅ |

## 8. Événements de fenêtre et de document

| Nom de l'événement JavaScript | Attribut HTML | Type | Description | Disponible dans fromEvent |
|---|---|---|---|---|
| load | onload | Event | Lorsque la page est complètement chargée | ✅ |
| resize | onresize | UIEvent | Lorsque la fenêtre est redimensionnée | ✅ |
| scroll | onscroll | Event | Lorsqu'une page défile | ✅ |
| unload | onunload | Event | Lorsque la page est fermée | ❌ |
| beforeunload | onbeforeunload | BeforeUnloadEvent | Juste avant la fermeture de la page | ❌ |
| error | onerror | ErrorEvent | Lorsqu'une erreur se produit | ✅ |
| visibilitychange | onvisibilitychange | Event | Lorsque l'état d'affichage de la page change (par ex., changement d'onglet) | ✅ |
| fullscreenchange | onfullscreenchange | Event | Lorsque l'état plein écran change | ✅ |

## 9. Événements média

| Nom de l'événement JavaScript | Attribut HTML | Type | Description | Disponible dans fromEvent |
|---|---|---|---|---|
| play | onplay | Event | Lorsque la lecture du média commence | ✅ |
| pause | onpause | Event | Lorsque la lecture du média est mise en pause | ✅ |
| ended | onended | Event | Lorsque la lecture du média se termine | ✅ |
| volumechange | onvolumechange | Event | Lorsque le volume du média est modifié | ✅ |
| seeking | onseeking | Event | Lorsque la recherche dans le média commence | ✅ |
| seeked | onseeked | Event | Lorsque la recherche dans le média est terminée | ✅ |
| timeupdate | ontimeupdate | Event | Lorsque le temps de lecture du média est mis à jour | ✅ |

## 10. Événements d'animation et de transition

| Nom de l'événement JavaScript | Attribut HTML | Type | Description | Disponible dans fromEvent |
|---|---|---|---|---|
| animationstart | onanimationstart | AnimationEvent | Lorsqu'une animation commence | ✅ |
| animationend | onanimationend | AnimationEvent | Lorsque l'animation se termine | ✅ |
| animationiteration | onanimationiteration | AnimationEvent | Lorsque l'animation est répétée | ✅ |
| transitionstart | ontransitionstart | TransitionEvent | Lorsqu'une transition CSS commence | ✅ |
| transitionend | ontransitionend | TransitionEvent | Lorsqu'une transition CSS se termine | ✅ |

## 11. Autres événements

| Nom de l'événement JavaScript | Attribut HTML | Type | Description | Disponible dans fromEvent |
|---|---|---|---|---|
| wheel | onwheel | WheelEvent | Lorsque la molette de la souris est tournée | ✅ |
| abort | onabort | UIEvent | Lorsque le chargement d'une ressource est interrompu | ✅ |
| hashchange | onhashchange | HashChangeEvent | Lorsque le hash de l'URL (par ex. `#section1`) est modifié | ✅ |
| message | onmessage | MessageEvent | Lorsqu'un message est reçu des Web Workers ou des iframes | ❌ |
| online | ononline | Event | Lorsque le réseau est de nouveau en ligne | ✅ |
| offline | onoffline | Event | Lorsque le réseau passe hors ligne | ✅ |
| popstate | onpopstate | PopStateEvent | Lorsqu'un changement d'état se produit via `history.pushState` ou `history.back` | ❌ |
| storage | onstorage | StorageEvent | Lorsque `localStorage` ou `sessionStorage` est modifié | ❌ |
| languagechange | onlanguagechange | Event | Lorsque le paramètre de langue est modifié (changement de paramètre du navigateur) | ❌ |
