---
description: L'operatore audit è un operatore di filtraggio RxJS che emette solo l'ultimo valore entro un periodo controllato da un Observable personalizzato. È ideale per il controllo dinamico del timing.
titleTemplate: ':title'
---

# audit - Ultimo valore su trigger

L'operatore `audit` attende che un Observable personalizzato emetta un valore ed emette l'**ultimo valore** dalla sorgente durante quel periodo.
Mentre `auditTime` controlla con un tempo fisso, `audit` può **controllare il periodo dinamicamente con un Observable**.

## 🔰 Sintassi e Utilizzo Base

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Evento click
const clicks$ = fromEvent(document, 'click');

// Periodo separato ogni secondo
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Click registrato');
});
```

- Quando si verifica un click, inizia un periodo di 1 secondo.
- Solo l'ultimo click durante quel secondo viene emesso.
- Il prossimo periodo inizia dopo 1 secondo.

[🌐 Documentazione Ufficiale RxJS - `audit`](https://rxjs.dev/api/operators/audit)

## 💡 Pattern di Utilizzo Tipici

- **Campionamento a intervalli dinamici**: Regola il periodo in base al carico
- **Controllo timing personalizzato**: Controllo del periodo basato su altri Observable
- **Limitazione eventi adattiva**: Diradamento in base alle circostanze

## 🔍 Differenza da auditTime

| Operatore | Controllo Periodo | Caso d'Uso |
|:---|:---|:---|
| `auditTime` | Tempo fisso (millisecondi) | Controllo semplice basato sul tempo |
| `audit` | **Observable personalizzato** | **Controllo dinamico del periodo** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - 1 secondo fisso
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('1 secondo fisso'));

// audit - Periodo dinamico
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // Periodo casuale 0-2 secondi
    return timer(period);
  })
).subscribe(() => console.log(`Periodo dinamico: ${period}ms`));
```

## 🧠 Esempio di Codice Pratico: Campionamento Dinamico in Base al Carico

Esempio di regolazione dell'intervallo di campionamento in base al carico del sistema.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// Crea UI
const output = document.createElement('div');
output.innerHTML = '<h3>Campionamento Dinamico</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Cambia Carico';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Livello di carico (0: basso, 1: medio, 2: alto)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Carico Basso', 'Carico Medio', 'Carico Alto'];
  statusDiv.textContent = `Carico attuale: ${levels[loadLevel]}`;
});

// Evento movimento mouse
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Regola il periodo in base al carico
    const periods = [2000, 1000, 500]; // Carico basso → periodo lungo, carico alto → periodo corto
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Posizione mouse: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Mostra massimo 10 elementi
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- Quando il carico è basso, dirada a intervalli di 2 secondi (modalità risparmio energetico)
- Quando il carico è alto, campiona finemente a intervalli di 500ms
- Il periodo può essere regolato dinamicamente in base al carico


## ⚠️ Note

### 1. Il Primo Valore Non Viene Emesso Immediatamente

`audit` attende fino alla fine del periodo dopo aver ricevuto il primo valore.

```ts
import { interval, timer } from 'rxjs';
import { audit, take } from 'rxjs';

interval(100).pipe(
  audit(() => timer(1000)),
  take(3)
).subscribe(val => {
  console.log(val);
});
// Output:
// 9  (dopo 1 secondo, ultimo valore di 0-9)
// 19 (dopo 2 secondi, ultimo valore di 10-19)
// 29 (dopo 3 secondi, ultimo valore di 20-29)
```

### 2. L'Observable di Durata Deve Essere Generato Nuovo Ogni Volta

La funzione passata ad `audit` deve **restituire un nuovo Observable ogni volta**.

```ts
// ❌ Esempio sbagliato: Riutilizza la stessa istanza Observable
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // Non funziona dalla 2a volta
).subscribe();

// ✅ Esempio corretto: Genera nuovo Observable ogni volta
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

## 🆚 Confronto con Operatori Simili

| Operatore | Timing Emissione | Valore Emesso | Caso d'Uso |
|:---|:---|:---|:---|
| `audit` | Alla **fine** del periodo | **Ultimo** valore nel periodo | Ottieni ultimo stato nel periodo |
| `throttle` | All'**inizio** del periodo | **Primo** valore nel periodo | Ottieni primo di eventi consecutivi |
| `debounce` | **Dopo il silenzio** | Valore appena prima del silenzio | Attendi completamento input |
| `sample` | **Quando un altro Observable si attiva** | Ultimo valore a quel momento | Snapshot periodici |


## 📚 Operatori Correlati

- **[auditTime](/it/guide/operators/filtering/auditTime)** - Controlla con tempo fisso (versione semplificata di `audit`)
- **[throttle](/it/guide/operators/filtering/throttleTime)** - Emetti primo valore all'inizio del periodo
- **[debounce](/it/guide/operators/filtering/debounceTime)** - Emetti valore dopo il silenzio
- **[sample](/it/guide/operators/filtering/sampleTime)** - Campiona al timing di un altro Observable

## Riepilogo

L'operatore `audit` emette l'ultimo valore entro un periodo controllato dinamicamente da un Observable personalizzato.

- ✅ Possibile controllo dinamico del periodo
- ✅ Campionamento adattivo in base al carico
- ✅ Controllo basato su altri stream
- ⚠️ Deve generare nuovo Observable ogni volta
- ⚠️ Attenzione alla memoria con emissioni frequenti
