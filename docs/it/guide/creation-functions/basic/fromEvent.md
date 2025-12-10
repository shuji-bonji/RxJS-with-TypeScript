---
description: "fromEvent() - Funzione di Creazione che converte eventi DOM ed EventEmitter in stream Observable con gestione automatica dei listener e prevenzione memory leak"
---

# fromEvent() - Converti Eventi in Observable

`fromEvent()` è una Funzione di Creazione che converte sorgenti di eventi come eventi DOM e Node.js EventEmitter in stream Observable.

## Panoramica

`fromEvent()` permette alle pipeline RxJS di gestire elaborazione asincrona basata su eventi. Registra automaticamente i listener di eventi quando sottoscritto e rimuove automaticamente i listener quando annullato, riducendo notevolmente il rischio di memory leak.

**Firma**:
```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

**Documentazione Ufficiale**: [📘 RxJS Ufficiale: fromEvent()](https://rxjs.dev/api/index/function/fromEvent)

## Uso Base

Questo è l'esempio più semplice di trattare eventi DOM come Observable.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Pulsante cliccato:', event);
});

// L'evento viene emesso ogni volta che clicchi
```

## Caratteristiche Importanti

### 1. Registrazione e Rimozione Automatica dei Listener

`fromEvent()` registra un listener di eventi alla subscription e rimuove automaticamente il listener all'annullamento.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent<MouseEvent>(document, 'click');

const subscription = clicks$.subscribe(event => {
  console.log('Posizione click:', event.clientX, event.clientY);
});

// Annulla subscription dopo 5 secondi (il listener eventi viene rimosso automaticamente)
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Subscription annullata');
}, 5000);
```

> [!IMPORTANT]
> **Prevenzione Memory Leak**
>
> Quando viene chiamato `unsubscribe()`, `removeEventListener()` viene eseguito automaticamente internamente. Questo elimina la necessità di rimuovere manualmente i listener e riduce notevolmente il rischio di memory leak.

### 2. Cold Observable (Ogni Subscription Registra Listener Indipendente)

L'Observable creato da `fromEvent()` è un **Cold Observable**. Ogni subscription registra un listener di eventi indipendente.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Subscription 1 - Registra listener A
clicks$.subscribe(() => console.log('Observer 1: Click'));

// Aggiungi subscription 2 dopo 1 secondo - Registra listener B indipendentemente
setTimeout(() => {
  clicks$.subscribe(() => console.log('Observer 2: Click'));
}, 1000);

// Entrambi i listener si attivano su un singolo click
// Questo prova che ogni subscription ha un listener indipendente
```

> [!NOTE]
> **Prova Cold Observable**
>
> Un nuovo listener di eventi viene registrato ogni volta che ti sottoscrivi e rimosso quando annulli. Questa è una caratteristica dei Cold Observable. Tuttavia, poiché la sorgente eventi (es., elemento DOM) è esterna e condivisa, ha anche la proprietà Hot di "non ricevere eventi prima della subscription".

### 3. Supporto Tipi TypeScript

I tipi di evento possono essere specificati esplicitamente.

```typescript
import { fromEvent } from 'rxjs';

const input = document.createElement('input');
input.type = 'text';
document.body.appendChild(input);
const input$ = fromEvent<InputEvent>(input, 'input');

input$.subscribe(event => {
  // Il tipo di event è InputEvent
  const target = event.target as HTMLInputElement;
  console.log('Valore input:', target.value);
});
```

### 4. Cold Observable

`fromEvent()` è un **Cold Observable**. Ogni subscription avvia un'esecuzione indipendente.

```typescript
import { fromEvent } from 'rxjs';

const button = document.createElement('button');
button.innerText = "Subscribe";
document.body.appendChild(button);

const clicks$ = fromEvent(document, 'click');

// Prima subscription - viene aggiunto listener eventi
clicks$.subscribe(() => console.log('Subscriber A'));

// Seconda subscription - viene aggiunto un altro listener eventi
clicks$.subscribe(() => console.log('Subscriber B'));

// Entrambi i listener si attivano quando clicchi una volta
// Output:
// Subscriber A
// Subscriber B
```

> [!NOTE]
> **Caratteristiche Cold Observable**:
> - Un'esecuzione indipendente viene avviata per ogni subscription
> - Ogni subscriber riceve il proprio stream di dati
> - Un listener eventi indipendente viene registrato per ogni subscription; unsubscribe rimuove automaticamente il listener
>
> Vedi [Cold Observable e Hot Observable](/it/guide/observables/cold-and-hot-observables) per maggiori informazioni.

## Casi d'Uso Pratici

### 1. Elaborazione Eventi Click

Controlla i click sui pulsanti e previeni click consecutivi.

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const button = document.createElement('button');
button.innerText = "submit";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  debounceTime(300), // Ignora click consecutivi entro 300ms
  map(() => 'Invio in corso...')
).subscribe(message => {
  console.log(message);
  // Elaborazione chiamata API, ecc.
});
```

### 2. Validazione Input Form in Tempo Reale

Stream eventi input ed esegui validazione in tempo reale.

```typescript
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'email: ';
const emailInput = document.createElement('input');
label.appendChild(emailInput);
document.body.appendChild(label);
const email$ = fromEvent<InputEvent>(emailInput, 'input');

email$.pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(500), // Elabora 500ms dopo che l'input si ferma
  distinctUntilChanged() // Solo quando il valore cambia
).subscribe(email => {
  console.log('Target validazione:', email);
  // Elaborazione validazione email
  validateEmail(email);
});

function validateEmail(email: string): void {
  const isValid = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  console.log(isValid ? 'Indirizzo email valido' : 'Indirizzo email non valido');
}
```

### 3. Implementazione Drag & Drop

Combina eventi mouse per implementare drag & drop.

```typescript
import { fromEvent } from 'rxjs';
import { switchMap, takeUntil, map } from 'rxjs';

// Crea elemento trascinabile
const element = document.createElement('div');
element.style.width = '100px';
element.style.height = '100px';
element.style.backgroundColor = '#333';
element.style.position = 'absolute'; // Imposta posizionamento assoluto
element.style.left = '50px'; // Posizione iniziale
element.style.top = '50px';
element.style.cursor = 'move'; // Cursore trascinabile
document.body.appendChild(element);

const mousedown$ = fromEvent<MouseEvent>(element, 'mousedown');
const mousemove$ = fromEvent<MouseEvent>(document, 'mousemove');
const mouseup$ = fromEvent<MouseEvent>(document, 'mouseup');

mousedown$.pipe(
  switchMap(startEvent => {
    // Registra posizione click dentro l'elemento
    const startX = startEvent.clientX - element.offsetLeft;
    const startY = startEvent.clientY - element.offsetTop;

    return mousemove$.pipe(
      map(moveEvent => ({
        left: moveEvent.clientX - startX,
        top: moveEvent.clientY - startY
      })),
      takeUntil(mouseup$) // Termina al rilascio del mouse
    );
  })
).subscribe(({ left, top }) => {
  // Aggiorna posizione elemento
  element.style.left = `${left}px`;
  element.style.top = `${top}px`;
});
```

### 4. Monitoraggio Eventi Scroll

Usato per tracciare infinite scrolling e posizione scroll.

```typescript
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

const scroll$ = fromEvent(window, 'scroll');

scroll$.pipe(
  throttleTime(200), // Elabora solo una volta ogni 200ms
  map(() => window.scrollY)
).subscribe(scrollPosition => {
  console.log('Posizione scroll:', scrollPosition);

  // Carica contenuto aggiuntivo quando raggiungi il fondo della pagina
  if (scrollPosition + window.innerHeight >= document.body.scrollHeight - 100) {
    console.log('Carica contenuto aggiuntivo');
    // loadMoreContent();
  }
});
```

## Uso in Pipeline

`fromEvent()` è ideale per l'elaborazione pipeline che parte da stream di eventi.

```typescript
import { fromEvent } from 'rxjs';
import { map, filter, scan } from 'rxjs';

const button = document.createElement('button');
button.innerText = "Counter";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  filter((event: Event) => {
    // Conta solo click mentre tieni premuto Shift
    return (event as MouseEvent).shiftKey;
  }),
  scan((count, _) => count + 1, 0),
  map(count => `Conteggio click: ${count}`)
).subscribe(message => console.log(message));
```

## Errori Comuni

### 1. Dimenticare di Annullare Subscription

#### ❌ Sbagliato - Dimenticare di annullare causa memory leak

```typescript
import { fromEvent } from 'rxjs';

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  clicks$.subscribe(console.log); // Non annullato!
}

setupEventListener();
```

#### ✅ Corretto - Annulla sempre subscription

```typescript
import { fromEvent } from 'rxjs';
import { Subscription } from 'rxjs';

let subscription: Subscription;

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  subscription = clicks$.subscribe(console.log);
}

function cleanup() {
  if (subscription) {
    subscription.unsubscribe();
  }
}

setupEventListener();
// Chiama cleanup() quando il componente viene distrutto, ecc.
```

> [!WARNING]
> **Attenzione ai Memory Leak**
>
> Nelle SPA e nei framework basati su componenti, assicurati di annullare la subscription quando distruggi un componente. Se dimentichi di annullare, i listener eventi rimarranno e causeranno memory leak.

### 2. Registrazione Duplicata di Listener Eventi Multipli

#### ❌ Sbagliato - Sottoscriversi allo stesso evento più volte registra più listener

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Observer 1'));
clicks$.subscribe(() => console.log('Observer 2'));
// Entrambi i log vengono visualizzati al click (due listener sono registrati)
```

#### ✅ Corretto - Multicast con share() se necessario

```typescript
import { fromEvent } from 'rxjs';
import { share } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(share());

clicks$.subscribe(() => console.log('Observer 1'));
clicks$.subscribe(() => console.log('Observer 2'));
// Un listener viene condiviso
```

## Considerazioni sulle Performance

Le performance dovrebbero essere considerate quando si gestiscono eventi che si attivano ad alta frequenza (scroll, mousemove, resize, ecc.).

> [!TIP]
> **Ottimizzazione di Eventi ad Alta Frequenza**:
> - `throttleTime()` - Elabora solo una volta per periodo di tempo
> - `debounceTime()` - Elabora dopo che l'input si ferma
> - `distinctUntilChanged()` - Elabora solo quando il valore cambia

#### ❌ Problema di Performance - Elabora ad ogni resize

```typescript
import { fromEvent } from 'rxjs';

const resize$ = fromEvent(window, 'resize');

resize$.subscribe(() => {
  console.log('Elaborazione resize'); // Elaborazione pesante
});
```

#### ✅ Ottimizzazione - Elabora solo una volta ogni 200ms

```typescript
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

const resize$ = fromEvent(window, 'resize');
resize$.pipe(
  throttleTime(200)
).subscribe(() => {
  console.log('Elaborazione resize'); // Riduzione carico
});
```

## Funzioni di Creazione Correlate

| Funzione | Differenza | Uso |
|----------|------|----------|
| **[from()](/it/guide/creation-functions/basic/from)** | Converti da array/Promise | Stream dati diversi da eventi |
| **[interval()](/it/guide/creation-functions/basic/interval)** | Emetti a intervalli regolari | Serve elaborazione periodica |
| **fromEventPattern()** | Registrazione eventi personalizzata | Sistemi di eventi personalizzati diversi da EventEmitter |

## Riepilogo

- `fromEvent()` converte eventi DOM ed EventEmitter in Observable
- Registra listener alla subscription, elimina automaticamente all'unsubscribe (previene memory leak)
- Funziona come Hot Observable
- Esegui sempre unsubscribe per prevenire memory leak
- Ottimizza eventi ad alta frequenza con `throttleTime()` e `debounceTime()`

## Prossimi Passi

- [interval() - Emetti Valori a Intervalli Regolari](/it/guide/creation-functions/basic/interval)
- [timer() - Inizia a Emettere Dopo un Ritardo](/it/guide/creation-functions/basic/timer)
- [Torna alle Funzioni di Creazione Base](/it/guide/creation-functions/basic/)
