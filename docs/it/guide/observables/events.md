---
description: "Spiega come usare fromEvent per gestire gli eventi DOM come Observable. Introduzione pratica allo streaming di clic, movimenti del mouse, input da tastiera e da form, all'implementazione del drag and drop, ai pattern di delega degli eventi e alla gestione type-safe degli eventi in TypeScript."
---

# Streaming degli eventi

Questa sezione fornisce un'introduzione completa alla creazione di Observable in RxJS, dalla sintassi di base alle applicazioni pratiche.

## Confronto tra gestione tradizionale degli eventi e RxJS

### Eventi click
#### ◇ Elaborazione convenzionale degli eventi DOM

```ts
document.addEventListener('click', (event) => {
  console.log('Cliccato:', event);
});

// Risultato:
// Cliccato: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

#### ◆ Elaborazione degli eventi con RxJS

```ts
import { fromEvent } from 'rxjs';

// Streaming dell'evento click
const clicks$ = fromEvent(document, 'click');
clicks$.subscribe(event => console.log('Click RxJS:', event));

// Risultato:
// Click RxJS: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Evento di movimento del mouse
#### ◇ Elaborazione convenzionale degli eventi DOM
```ts
document.addEventListener('mousemove', (event) => {
  console.log('Posizione mouse:', event.clientX, event.clientY);
});

// Risultato:
// Posizione mouse: 4 357
// Posizione mouse: 879 148
// Posizione mouse: 879 148
```

#### ◆ Elaborazione degli eventi con RxJS

```ts
import { fromEvent } from 'rxjs';
import { map, throttleTime } from 'rxjs';

// Streaming dell'evento di movimento del mouse (con throttling)
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  throttleTime(100), // Limitato ogni 100 millisecondi
  map(event => ({ x: event.clientX, y: event.clientY }))
);
mouseMove$.subscribe(position => console.log('Posizione mouse:', position));

// Risultato:
// Posizione mouse: {x: 177, y: 453}
// Posizione mouse: {x: 1239, y: 297}
```

### Eventi della tastiera
#### ◇ Elaborazione convenzionale degli eventi DOM
```ts
document.addEventListener('keydown', (event) => {
  console.log('Tasto premuto:', event.key);
});

// Risultato:
// Tasto premuto: h
// Tasto premuto: o
// Tasto premuto: g
// Tasto premuto: e
```

#### ◆ Elaborazione degli eventi con RxJS

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Streaming dell'evento della tastiera
const keyDown$ = fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  filter(key => key.length === 1) // Solo caratteri singoli (esclusi i tasti modificatori)
);
keyDown$.subscribe(key => console.log('Tasto premuto:', key));

// Risultato:
// Tasto premuto: h
// Tasto premuto: o
// Tasto premuto: g
// Tasto premuto: e
```


## Come usare e applicare fromEvent

`fromEvent` è il modo più comune per convertire gli eventi DOM in Observable. `fromEvent` è la funzione di conversione evento → Observable più elementare ed è il punto di partenza per la gestione degli eventi con RxJS.

### Utilizzo di base
```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe((event) => console.log('Click RxJS:', event));

// Risultato:
// Click RxJS: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```


### Specificare il target e il tipo dell'evento
```ts
import { fromEvent } from 'rxjs';

const myButton = document.querySelector('#myButton')!;
const buttonClicks$ = fromEvent<MouseEvent>(myButton, 'click');
buttonClicks$.subscribe((event) => console.log('Click myButton:', event));

// Risultato:
// Click myButton: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

### Specificazione delle opzioni (ascolto in fase di cattura)
```ts
import { fromEvent } from 'rxjs';

const capturedClicks$ = fromEvent(document, 'click', { capture: true });
capturedClicks$.subscribe((event) => console.log('Click pagina:', event));

// Risultato:
// Click pagina: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!NOTE]
> Esistono due fasi di propagazione degli eventi DOM: "capture" e "bubbling".
> Normalmente è "bubbling" (gli eventi si propagano dall'elemento figlio all'elemento genitore), ma se si specifica `capture: true`, si ascolta nella "fase di cattura" (propagazione dall'elemento genitore all'elemento figlio).
> Questo permette agli eventi di essere rilevati dall'elemento genitore prima che vengano elaborati dall'elemento figlio.

## Gestione di più sorgenti di eventi

RxJS consente di unire più fonti di eventi in una logica comune tramite `merge` o `combineLatest`.

```ts
import { fromEvent, merge } from 'rxjs';
import { map } from 'rxjs';

// Unione dei clic da più pulsanti
const button1Clicks$ = fromEvent(document.querySelector('#button1')!, 'click')
  .pipe(map(() => 'Pulsante 1 cliccato'));

const button2Clicks$ = fromEvent(document.querySelector('#button2')!, 'click')
  .pipe(map(() => 'Pulsante 2 cliccato'));

// Merge di entrambi gli stream di eventi
const allButtonClicks$ = merge(button1Clicks$, button2Clicks$);
allButtonClicks$.subscribe(message => console.log(message));
```

#### Risultato dell'esecuzione
```
Pulsante 1 cliccato
```
```
Pulsante 2 cliccato
```


## Conversione e manipolazione del flusso di eventi

Il vantaggio degli eventi in streaming è che possono essere facilmente convertiti e manipolati con gli operatori di RxJS.

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  debounceTime,
  distinctUntilChanged,
} from 'rxjs';

// Monitoraggio delle modifiche al valore del campo di input
const input$ = fromEvent<InputEvent>(
  document.querySelector('#searchInput')!,
  'input'
).pipe(
  map((event) => (event.target as HTMLInputElement).value),
  filter((text) => text.length > 2), // Elabora solo se 3 o più caratteri
  debounceTime(300), // Attende 300ms di intervallo (non si attiva durante la digitazione)
  distinctUntilChanged() // Non si attiva se il valore è lo stesso di prima
);

input$.subscribe((searchText) => {
  console.log('Testo di ricerca:', searchText);
  // Qui si può chiamare l'API di ricerca
});

```

#### Risultato dell'esecuzione
```sh
Testo di ricerca: abc
Testo di ricerca: abcd
```
In questo modo, la reattività e la manutenibilità dell'interfaccia utente possono essere notevolmente migliorate gestendo gli eventi di input come stream.

## Esempio di implementazione del drag and drop

Come esempio di utilizzo di una combinazione di più eventi, proviamo a gestire le operazioni di trascinamento del mouse con Observable.

```ts
import { fromEvent } from 'rxjs';
import { map, switchMap, takeUntil, tap } from 'rxjs';

function implementDragAndDrop(element: HTMLElement) {
  // Stream dell'evento mousedown
  const mouseDown$ = fromEvent<MouseEvent>(element, 'mousedown');

  // Stream dell'evento mousemove sul documento
  const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');

  // Stream dell'evento mouseup sul documento
  const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

  // Processo di trascinamento
  const drag$ = mouseDown$.pipe(
    tap(event => {
      // Previene il comportamento di trascinamento predefinito del browser
      event.preventDefault();
    }),
    switchMap(startEvent => {
      // Registra la posizione iniziale
      const initialX = startEvent.clientX;
      const initialY = startEvent.clientY;
      const elementX = parseInt(element.style.left || '0', 10);
      const elementY = parseInt(element.style.top || '0', 10);

      // Restituisce lo stream di movimento del mouse (fino a mouseUp)
      return mouseMove$.pipe(
        map(moveEvent => ({
          x: elementX + (moveEvent.clientX - initialX),
          y: elementY + (moveEvent.clientY - initialY)
        })),
        takeUntil(mouseUp$) // Termina con mouseup
      );
    })
  );

  // Sottoscrivi e aggiorna la posizione
  drag$.subscribe(position => {
    element.style.left = `${position.x}px`;
    element.style.top = `${position.y}px`;
    console.log(`${element.style.left}, ${element.style.top}`);
  });
}

// Esempio di utilizzo
const draggableElement = document.querySelector('#draggable') as HTMLElement;
implementDragAndDrop(draggableElement);
```

#### Risultato dell'esecuzione
```
1px, 0px
1px, -1px
0px, -2px
0px, -3px
0px, -4px
```

## Monitoraggio e validazione degli input del form

I processi tipici dell'interfaccia utente, come la validazione dei form, possono essere scritti in modo più dichiarativo e sicuro con Observable.

```ts
import { fromEvent, combineLatest } from 'rxjs';
import { map, startWith, debounceTime } from 'rxjs';

function validateForm() {
  // Riferimenti ai campi di input
  const usernameInput = document.querySelector('#username') as HTMLInputElement;
  const passwordInput = document.querySelector('#password') as HTMLInputElement;
  const submitButton = document.querySelector('#submit') as HTMLButtonElement;

  // Stream delle modifiche ai campi di input
  const username$ = fromEvent<InputEvent>(usernameInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // Valore iniziale
  );

  const password$ = fromEvent<InputEvent>(passwordInput, 'input').pipe(
    map(e => (e.target as HTMLInputElement).value),
    startWith('') // Valore iniziale
  );

  // Combina entrambi gli input per la validazione
  const formValid$ = combineLatest([username$, password$]).pipe(
    debounceTime(300),
    map(([username, password]) => {
      return username.length >= 3 && password.length >= 6;
    })
  );

  // Attiva/disattiva il pulsante in base allo stato di validazione del form
  formValid$.subscribe(isValid => {
    submitButton.disabled = !isValid;
  });

  // Gestione dell'invio del form
  const submit$ = fromEvent(submitButton, 'click');
  submit$.subscribe(() => {
    console.log('Invio form:', {
      username: usernameInput.value,
      password: passwordInput.value
    });
    // Qui si esegue l'effettivo processo di invio
  });
}

// Esempio di utilizzo
validateForm();
```
#### Risultato dell'esecuzione
```
Invio form: {username: 'testuser', password: '123456'}
```

## Link all'elenco degli eventi

Per un elenco di tutti gli eventi e della loro disponibilità in `fromEvent`, si può trovare il seguente link.

➡️ **[Elenco degli eventi](./events-list.md)**

Questo elenco è utile per la programmazione reattiva con RxJS, in quanto mostra chiaramente se gli eventi JavaScript standard sono supportati da `fromEvent`.


## Eventi non disponibili in fromEvent {#cannot-used-fromEvent}

`fromEvent` si basa sull'interfaccia `EventTarget` del DOM. Pertanto, i seguenti eventi non possono essere gestiti direttamente da `fromEvent`. Questo perché sono legati a oggetti specifici o hanno i loro listener di eventi.

| Nome evento | Tipo | Motivo |
| ------------------ | --------------------- | ------------------------------------------------ |
| `beforeunload` | `BeforeUnloadEvent` | Evento eseguito prima della chiusura della finestra, dipende dal comportamento del browser piuttosto che dai listener di eventi DOM |
| `unload` | `Event` | Non valido per RxJS Observable, perché il listener viene rimosso quando la pagina viene chiusa completamente |
| `message` | `MessageEvent` | I messaggi di ServiceWorker e WebWorker non possono essere catturati direttamente con `fromEvent` |
| `popstate` | `PopStateEvent` | Le modifiche a `history.pushState` o `replaceState` devono essere gestite manualmente |
| `storage` | `StorageEvent` | Le modifiche a `localStorage` non possono essere monitorate tramite `fromEvent` (richiede `window.addEventListener`) |
| `languagechange` | `Event` | Le modifiche alle impostazioni del browser dipendono dal comportamento dell'oggetto `window` |
| `fetch` | `Event` | L'avanzamento del `fetch` (es. `onprogress`) non è un normale evento DOM |
| `WebSocket` | `Event` | `onmessage`, `onopen`, `onclose` hanno i loro listener di eventi |
| `ServiceWorker` | `Event` | `message`, `install`, `activate` ecc. non possono essere gestiti da `fromEvent` |

### Alternative
Se si desidera monitorare questi eventi, utilizzare i seguenti metodi.

- `window.addEventListener('message', callback)`
- `window.addEventListener('popstate', callback)`
- `window.addEventListener('storage', callback)`
- Per `WebSocket`: `ws.addEventListener('message', callback)`
- Per `ServiceWorker`: `navigator.serviceWorker.addEventListener('message', callback)`

Quando si avvolge in RxJS, invece di `fromEvent` si può generare manualmente un Observable come segue.

```typescript
import { Observable } from 'rxjs';

const message$ = new Observable<MessageEvent>(observer => {
  const handler = (event: MessageEvent) => observer.next(event);
  window.addEventListener('message', handler);

  // Processo di cleanup
  return () => window.removeEventListener('message', handler);
});

message$.subscribe(event => {
  console.log('Messaggio ricevuto:', event.data);
});
```

## Riepilogo e best practice

Questo articolo ha esaminato i vantaggi e le applicazioni specifiche di rendere gli eventi Observable.

La gestione degli eventi con RxJS offre i seguenti vantaggi:

- Gestione dichiarativa e strutturata degli eventi
- Facilità di filtraggio, trasformazione e ritardo degli eventi tramite `pipe()` e operatori
- Integrazione di più sorgenti di eventi e controllo di stati complessi chiaramente esprimibili
- Gestione centralizzata degli effetti collaterali tramite `subscribe`

### Best Practice

- `fromEvent` per ogni componente UI dovrebbe essere correttamente `unsubscribe` (usando `takeUntil` ecc.)
- I riferimenti DOM dovrebbero essere stabilizzati con null-check e `!` esplicito
- Gli stream dovrebbero essere divisi finemente e si dovrebbe essere consapevoli della differenza tra `switchMap` e `mergeMap`
- La combinazione con la comunicazione backend può essere controllata con `exhaustMap`, `concatMap`, ecc.

Lo streaming degli eventi con RxJS va oltre la semplice elaborazione di clic e keydown e rappresenta **il concetto di progettazione di base dell'intera costruzione dell'interfaccia utente reattiva**.
