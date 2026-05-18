---
description: "auditTime è un operatore di filtraggio di RxJS che attende un tempo specificato per l'emissione di un valore e fornisce l'ultimo valore entro tale periodo. Viene utilizzato soprattutto quando si desidera campionare periodicamente l'ultimo stato di eventi ad alta frequenza, come il tracciamento della posizione dello scroll, il ridimensionamento della finestra, il movimento del mouse, ecc."
---

# auditTime - ultimo valore emesso dopo l'ora specificata

L'operatore `auditTime` attende un **tempo specificato** dopo l'emissione di un valore e produce l'**ultimo valore** entro tale periodo. Quindi attende l'arrivo del valore successivo.

## 🔰 Sintassi e utilizzo di base

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Fare clic.！'));
```

**Flusso delle operazioni**:.
1. si verifica il primo clic
2. attende 1 secondo (i clic durante questo tempo vengono registrati ma non emessi)
3. emette l'ultimo clic dopo 1 secondo
Attendere il clic successivo

[🌐 Documentazione ufficiale RxJS - auditTime](https://rxjs.dev/api/operators/auditTime)

## 🆚 Contrasto con throttleTime

throttleTime e auditTime sono simili, ma differiscono per i valori che producono.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: Emissione del primo valore
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Uscita.: 0, 4, 8(primo valore di ogni periodo)

// auditTime: Uscita dell'ultimo valore
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Uscita.: 3, 6, 9(ultimo valore di ciascun periodo)
```

**Confronto tra linee temporali**:.

```
Fonte:     0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (Primo)   (Primo)   (Primo)

audit:      -------3--------6--------9----|
                  (Ultimo)   (Ultimo)   (Ultimo)
```

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Fare clic.！'));
```

## 💡 Modello di utilizzo tipico

1. **Ottimizzare il ridimensionamento della finestra**.

```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // 200msOttenere l'ultima dimensione nell'intervallo
   ).subscribe(() => {
     console.log(`Dimensione della finestra: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **Tracciamento della posizione di scorrimento**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map } from 'rxjs';

   fromEvent(window, 'scroll').pipe(
     auditTime(100),
     map(() => ({
       scrollY: window.scrollY,
       scrollX: window.scrollX
     }))
   ).subscribe(position => {
     console.log(`Posizione di scorrimento: Y=${position.scrollY}, X=${position.scrollX}`);
   });
   ```

3. **Movimento di trascinamento fluido**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map, takeUntil, switchMap } from 'rxjs';

   // Creare elementi trascinabili
   const box = document.createElement('div');
   box.style.width = '100px';
   box.style.height = '100px';
   box.style.backgroundColor = '#3498db';
   box.style.position = 'absolute';
   box.style.cursor = 'move';
   box.style.left = '100px';
   box.style.top = '100px';
   box.textContent = 'Trascinamento';
   box.style.display = 'flex';
   box.style.alignItems = 'center';
   box.style.justifyContent = 'center';
   box.style.color = 'white';
   document.body.appendChild(box);

   const mouseDown$ = fromEvent<MouseEvent>(box, 'mousedown');
   const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');
   const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

   // Implementare le operazioni di trascinamento
   mouseDown$.pipe(
     switchMap(startEvent => {
       const startX = startEvent.clientX - box.offsetLeft;
       const startY = startEvent.clientY - box.offsetTop;

       return mouseMove$.pipe(
         auditTime(16), // Approssimazione.60FPS(vedere anche16ms) per aggiornare la posizione
         map(moveEvent => ({
           x: moveEvent.clientX - startX,
           y: moveEvent.clientY - startY
         })),
         takeUntil(mouseUp$)
       );
     })
   ).subscribe(position => {
     box.style.left = `${position.x}px`;
     box.style.top = `${position.y}px`;
   });
   ```

## 🧠 Esempio pratico di codice (tracciamento del mouse)

Questo esempio traccia i movimenti del mouse e visualizza l'ultima posizione a intervalli regolari.

```

ts.
import { fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

// Creare gli elementi dell'interfaccia utente
const container = document.createElement('div');.
container.style.height = '300px';
container.style.border = '2px solid #3498db';
container.style.padding = '20px';
container.style.position = 'relativo';
container.textContent = 'Muovi il mouse in quest'area';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
positionDisplay.style.fontFamily = 'monospace';
document.body.appendChild(positionDisplay);

const dot = document.createElement('div');
dot.style.width = '10px';
dot.style.height = '10px';
dot.style.borderRadius = '50%';
dot.style.backgroundColor = '#e74c3c';
dot.style.position = 'absolute';
dot.style.display = 'none';
container.appendChild(dot);

// Evento di spostamento del mouse
fromEvent\<MouseEvent>(container, 'mousemove').pipe(
  map(evento => {
    const rect = container.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,.
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // Ottiene l'ultima posizione ogni 100ms
).subscribe(posizione => {
  positionDisplay.textContent = `Ultima posizione (ogni 100 ms): X=${posizione.x.toFixed(0)}, Y=${posizione.y.toFixed(0)}`;

  // Sposta il punto nell'ultima posizione
  dot.style.left = `${position.x - 5}px`;
  dot.style.top = `${position.y - 5}px`;
  dot.style.display = 'block';
});

```

Questo codice recupera e visualizza l'ultima posizione solo ogni volta che il mouse viene spostato, anche se il mouse viene spostato frequentemente,100msIl codice recupera e visualizza l'ultima posizione solo per ogni movimento del mouse.

## 🎯 debounceTime Le differenze tra

`auditTime` e `debounceTime` è che**entrambi producono l'ultimo valore**ma il codice**La tempistica è completamente diversa**l'ultimo valore viene emesso.

### La differenza decisiva

| Operatore | operazione | utilizzo del sistema in modi diversi |
|---|---|---|
| `auditTime(ms)` | Quando arriva un valore**msSempre in uscita dopo**(anche se l'ingresso continua) | Campionamento periodico |
| `debounceTime(ms)` | **Dopo che l'ingresso si è fermato**msUscita dopo | Attendere il completamento dell'ingresso |

### Esempi specifici：Differenze nell'input di ricerca

```

ts.
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Input parola di ricerca';
document.body.appendChild(input);

// AuditTime: Eseguire la ricerca ogni 300 ms anche durante l'inserimento di input
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → Ricerca:', input.value);
});

// debounceTime: attendere 300 ms dopo che l'input si è fermato, quindi eseguire la ricerca
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → Ricerca:', input.value);
});

```

### Differenze viste nella timeline

Differenza vista quando un utente fa clic su "ab'→'abc'→'abcd' durante la digitazione veloce:

```

Evento di input: a--b--c--d------------|
              ↓
auditTime: ------c-----d----------|
            (dopo 300 ms) (dopo 300 ms)
            → Cerca 'abc', cerca 'abcd' (2 volte in totale)

debounceTime: --------------------d-|
                              (300 ms dopo l'arresto)
            → Cerca "abcd" (solo una volta in totale)

```

**Facile da ricordare**:
- **`auditTime`**: 'Controllato regolarmente (audit)"→ 'Controllate sempre a intervalli regolari'
- **`debounceTime`**: 'Aspettare che si sia calmato (...)'.debounceAspettare che sia tranquillo.→ 'Aspettare finché non è tranquillo'

### Uso pratico

```

ts.
// ✅ AuditTime se appropriato
// - Tracciamento della posizione di scorrimento (vogliamo ottenerla periodicamente, anche se stiamo scorrendo tutto il tempo)
fromEvent(window, 'scroll').pipe(
  auditTime(100) // ottiene l'ultima posizione ogni 100 ms
).subscribe(/* ... */);

// ✅ se debounceTime è appropriato.
// - casella di ricerca (vogliamo effettuare la ricerca al termine dell'input)
fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // aspetta 300 ms dopo l'interruzione dell'input
).subscribe(/* ... */);

```

## 📋 Utilizzo sicuro per i tipi

TypeScript Questo è un esempio di implementazione type-safe che fa uso dei generici in

```

ts.
import { Observable, fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

interfaccia MousePosition {
  x: numero;
  y: numero
  timestamp: numero; }
}

function trackMousePosition(
  elemento: HTMLElement,.
  intervalli: numero
): Observable {
  return fromEvent\<MouseEvent>(element, 'mousemove').pipe(
    auditTime(intervalli),.
    map(event => ({ {
      x: event.clientX, event.
      y: event.clientY,.
      timestamp: Date.now())
    } come MousePosition))
  );

// Esempio di utilizzo
const canvas = document.createElement('div');
canvas.style.width = '400px';
canvas.style.height = '300px';
canvas.style.border = '1px solid black';
document.body.appendChild(canvas);

trackMousePosition(canvas, 200).subscribe(position => {
  console.log(`Posizione: (${posizione.x}, ${posizione.y}) a ${posizione.timestamp}`);
});

```

## 🔄 auditTime e throttleTime Combinazione di

In alcuni scenari, entrambi possono essere combinati.

```

ts.
import { interval } from 'rxjs';
importare { throttleTime, auditTime, take } da 'rxjs';

const source$ = interval(100).pipe(take(50));.

// ordine di throttleTime → auditTime
source$.pipe(
  throttleTime(1000), // passa il primo valore ogni secondo
  auditTime(500) // quindi attende 500 ms e invia l'ultimo valore
).subscribe(console.log);.

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Fare clic.！'));

ts.
import { fromEvent } da 'rxjs';
import { auditTime } from 'rxjs';

// Creare un campo di input per la ricerca
const input = document.createElement('input');.
input.type = 'text';
input.placeholder = 'Cerca...' ;
document.body.appendChild(input);

// ❌ Cattivo esempio: usare auditTime per l'input di ricerca
fromEvent(input, 'input').pipe(
  auditTime(300) // la ricerca viene eseguita ogni 300 ms durante l'input
).subscribe(() => {
  console.log('Ricerca eseguita');
});

```

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Fare clic.！'));
```

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Clic!'));
```

ts.
import { fromEvent } from 'rxjs';
import { debounceTime } from 'rxjs';

// Creare un campo di input per la ricerca
const input = document.createElement('input');.
input.type = 'text';
input.placeholder = 'Cerca...' ;
document.body.appendChild(input);

// ✅ Buon esempio: usare debounceTime per l'input di ricerca
fromEvent(input, 'input').pipe(
  debounceTime(300) // Attendere 300 ms dopo l'arresto dell'input prima di eseguire la ricerca
).subscribe(() => {
  console.log('Ricerca eseguita', input.value);
});
```

## 🎓 Riepilogo

### Quando si dovrebbe usare auditTime.
- ✅ Quando sono richiesti valori aggiornati a intervalli regolari
- ✅ Eventi ad alta frequenza come scorrimento, ridimensionamento, movimento del mouse
- ✅ Quando è richiesto un campionamento periodico
- ✅ Quando si desidera riflettere lo stato più recente.

### Quando si deve usare throttleTime.
- ✅ Quando è richiesta una risposta immediata
- ✅ Se si vuole iniziare l'elaborazione con il primo valore
- ✅ Prevenzione dello schiacciamento dei tasti

### Quando utilizzare debounceTime.
- ✅ Se si desidera attendere il completamento dell'input
- ✅ Ricerca, completamento automatico
- ✅ Aspettare che l'utente smetta di digitare.

### Note.
- ⚠️ L'auditTime produce solo l'ultimo valore del periodo (i valori intermedi vengono scartati).
- ⚠️ Non è molto efficace se impostato per intervalli brevi.
- ⚠️ throttleTime` o debounceTime` possono essere più appropriati a seconda dell'applicazione.

## 🚀 Prossimi passi.

- **[throttleTime](. /throttleTime)** - imparare a passare il primo valore.
- **[debounceTime](. /debounceTime)** - imparare a emettere valori dopo l'interruzione dell'input.
- **[filter](. /filter)** - impara a filtrare in base a delle condizioni.
- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - per imparare a utilizzare casi d'uso reali.
