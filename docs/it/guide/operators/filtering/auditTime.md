---
description: auditTime è un operatore di filtraggio RxJS che attende un tempo specificato dopo l'emissione di un valore e restituisce l'ultimo valore entro quel periodo. È ideale quando vuoi campionare periodicamente l'ultimo stato di eventi ad alta frequenza come tracciamento posizione scroll, ridimensionamento finestra e movimento mouse. È importante capire la differenza da throttleTime e debounceTime e usarli appropriatamente.
titleTemplate: ':title | RxJS'
---

# auditTime - Ultimo valore per periodo

L'operatore `auditTime` attende un **tempo specificato** dopo l'emissione di un valore e restituisce l'**ultimo valore** entro quel periodo di tempo. Poi attende il prossimo valore.


## 🔰 Sintassi e Utilizzo Base

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Click!'));
```

**Flusso di operazione**:
1. Si verifica il primo click
2. Attendi 1 secondo (i click durante questo tempo sono registrati ma non emessi)
3. Emetti l'ultimo click dopo 1 secondo
4. Attendi il prossimo click

[🌐 Documentazione Ufficiale RxJS - `auditTime`](https://rxjs.dev/api/operators/auditTime)


## 🆚 Confronto con throttleTime

`throttleTime` e `auditTime` sono simili, ma emettono valori diversi.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: Emetti il primo valore
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Output: 0, 4, 8 (primo valore di ogni periodo)

// auditTime: Emetti l'ultimo valore
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Output: 3, 6, 9 (ultimo valore di ogni periodo)
```

**Confronto timeline**:
```
Sorgente:   0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (primo)  (primo)  (primo)

audit:      -------3--------6--------9----|
                  (ultimo)  (ultimo) (ultimo)
```

| Operatore | Valore Emesso | Timing Emissione | Caso d'Uso |
|---|---|---|---|
| `throttleTime(ms)` | **Primo** valore nel periodo | Alla ricezione del valore | Reazione immediata necessaria |
| `auditTime(ms)` | **Ultimo** valore nel periodo | Alla fine del periodo | Ultimo stato necessario |
| `debounceTime(ms)` | **Ultimo** valore dopo silenzio | Dopo che l'input si ferma | Attendi completamento input |


## 💡 Pattern di Utilizzo Tipici

1. **Ottimizzazione Ridimensionamento Finestra**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // Ottieni ultima dimensione ogni 200ms
   ).subscribe(() => {
     console.log(`Dimensione finestra: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **Tracciamento Posizione Scroll**
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
     console.log(`Posizione scroll: Y=${position.scrollY}, X=${position.scrollX}`);
   });
   ```


## 🎯 Differenza da debounceTime

`auditTime` e `debounceTime` entrambi **emettono l'ultimo valore**, ma il **timing è completamente diverso**.

### Differenza Chiave

| Operatore | Comportamento | Caso d'Uso |
|---|---|---|
| `auditTime(ms)` | **Emette sempre dopo ms** una volta arrivato il valore (anche se l'input continua) | Vuoi campionare periodicamente |
| `debounceTime(ms)` | Emette dopo ms **dopo che l'input si ferma** | Vuoi attendere il completamento dell'input |

### Esempio Concreto: Differenza nell'Input di Ricerca

```ts
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Inserisci parole chiave di ricerca';
document.body.appendChild(input);

// auditTime: Esegui ricerca ogni 300ms anche mentre si digita
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → Ricerca:', input.value);
});

// debounceTime: Esegui ricerca 300ms dopo che si smette di digitare
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → Ricerca:', input.value);
});
```

### Visualizzazione Timeline

Quando l'utente digita "ab" → "abc" → "abcd" rapidamente:

```
Eventi input:   a--b--c--d------------|
              ↓
auditTime:    ------c-----d----------|
            (dopo 300ms) (dopo 300ms)
            → Ricerca "abc", ricerca "abcd" (2 volte totali)

debounceTime: --------------------d-|
                              (300ms dopo stop)
            → Ricerca "abcd" (1 volta sola)
```

**Promemoria Facile**:
- **`auditTime`**: "Audit periodico" → Controlla a intervalli regolari
- **`debounceTime`**: "Attendi finché si stabilizza (debounce)" → Attendi finché è calmo


## 🧠 Esempio di Codice Pratico (Tracciamento Mouse)

Esempio di tracciamento del movimento mouse e visualizzazione dell'ultima posizione a intervalli regolari.

```ts
import { fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

// Crea elementi UI
const container = document.createElement('div');
container.style.height = '300px';
container.style.border = '2px solid #3498db';
container.style.padding = '20px';
container.style.position = 'relative';
container.textContent = 'Muovi il mouse in questa area';
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

// Evento movimento mouse
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => {
    const rect = container.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // Ottieni ultima posizione ogni 100ms
).subscribe(position => {
  positionDisplay.textContent = `Ultima posizione (intervallo 100ms): X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}`;

  // Muovi il punto all'ultima posizione
  dot.style.left = `${position.x - 5}px`;
  dot.style.top = `${position.y - 5}px`;
  dot.style.display = 'block';
});
```

Questo codice ottiene e visualizza solo l'ultima posizione ogni 100ms, anche quando il mouse si muove frequentemente.


## 🎓 Riepilogo

### Quando Usare auditTime
- ✅ Quando hai bisogno dell'ultimo valore a intervalli regolari
- ✅ Eventi ad alta frequenza come scroll, resize, movimento mouse
- ✅ Quando è necessario il campionamento periodico
- ✅ Quando vuoi riflettere l'ultimo stato

### Quando Usare throttleTime
- ✅ Quando è necessaria una reazione immediata
- ✅ Quando vuoi iniziare l'elaborazione con il primo valore
- ✅ Prevenire click multipli sul pulsante

### Quando Usare debounceTime
- ✅ Quando vuoi attendere il completamento dell'input
- ✅ Ricerca, autocomplete
- ✅ Attendere finché l'utente smette di digitare

### Note
- ⚠️ `auditTime` emette solo l'ultimo valore nel periodo (i valori intermedi sono scartati)
- ⚠️ Se impostato su un intervallo breve, potrebbe non essere molto efficace
- ⚠️ A seconda del caso d'uso, `throttleTime` o `debounceTime` potrebbero essere più appropriati


## 🚀 Prossimi Passi

- **[throttleTime](/it/guide/operators/filtering/throttleTime)** - Impara come far passare il primo valore
- **[debounceTime](/it/guide/operators/filtering/debounceTime)** - Impara come emettere valori dopo che l'input si ferma
- **[filter](/it/guide/operators/filtering/filter)** - Impara come filtrare in base a condizioni
- **[Esempi Pratici Operatori di Filtraggio](/it/guide/operators/filtering/practical-use-cases)** - Impara casi d'uso reali
