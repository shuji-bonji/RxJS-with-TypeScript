---
description: L'operatore sampleTime è un operatore di filtraggio RxJS che campiona periodicamente l'ultimo valore da uno stream a intervalli di tempo specificati. È ideale per scattare snapshot periodici.
titleTemplate: ':title | RxJS'
---

# sampleTime - Campiona l'Ultimo Valore a Intervalli di Tempo Specificati

L'operatore `sampleTime` **campiona periodicamente** ed emette l'**ultimo valore** dall'Observable sorgente a **intervalli di tempo specificati**.
Come snapshot periodici, ottiene l'ultimo valore a quel momento.

## 🔰 Sintassi e Utilizzo Base

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Campiona ogni 2 secondi');
});
```

**Flusso di operazione**:
1. Il timer si attiva periodicamente ogni 2 secondi
2. Se c'è un ultimo evento click a quel momento, emettilo
3. Se non c'è valore durante il periodo di campionamento, non emette nulla

[🌐 Documentazione Ufficiale RxJS - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

## 💡 Pattern di Utilizzo Tipici

- **Acquisizione periodica dati sensore**: Ultime informazioni su temperatura o posizione ogni secondo
- **Dashboard real-time**: Aggiornamenti di stato periodici
- **Monitoraggio prestazioni**: Raccolta metriche a intervalli regolari
- **Elaborazione frame gioco**: Campionamento periodico per controllo FPS

## 🧠 Esempio di Codice Pratico: Campionamento Periodico Posizione Mouse

Esempio di campionamento e visualizzazione posizione mouse ogni secondo.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// Crea UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Campionamento Posizione Mouse (ogni secondo)';
container.appendChild(title);

const area = document.createElement('div');
area.style.width = '100%';
area.style.height = '300px';
area.style.border = '2px solid #4CAF50';
area.style.backgroundColor = '#f5f5f5';
area.style.display = 'flex';
area.style.alignItems = 'center';
area.style.justifyContent = 'center';
area.style.fontSize = '18px';
area.textContent = 'Muovi il mouse in questa area';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// Evento movimento mouse
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // Campiona ogni secondo
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>Campione #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    Posizione: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Mostra massimo 10 elementi
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- Anche se continui a muovere il mouse, solo l'ultima posizione a quel momento viene campionata ogni secondo.
- Se non muovi il mouse per 1 secondo, non emette nulla durante quel periodo.

## 🆚 Confronto con Operatori Simili

### sampleTime vs throttleTime vs auditTime

| Operatore | Timing Attivazione | Valore Emesso | Caso d'Uso |
|:---|:---|:---|:---|
| `sampleTime(1000)` | **Timing regolare ogni 1 secondo** | Ultimo valore a quel momento | Snapshot periodici |
| `throttleTime(1000)` | Ignora per 1 secondo dopo ricezione valore | Primo valore all'inizio del periodo | Diradamento eventi |
| `auditTime(1000)` | 1 secondo dopo ricezione valore | Ultimo valore nel periodo | Ultimo stato nel periodo |

**Differenza Visiva**:

```
Input: --|1|2|3|---|4|5|6|---|7|8|9|
      0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (campiona periodicamente)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (passa primo e ignora durante periodo)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (ultimo valore alla fine del periodo)
```

## ⚠️ Note

### 1. Quando Non C'è Valore Durante il Periodo di Campionamento

Se non c'è nuovo valore al timing del campionamento, non emette nulla.

### 2. Attendi Fino al Primo Timing di Campionamento

`sampleTime` non emette nulla finché non è trascorso il tempo specificato.

### 3. Timing di Completamento

Anche se la sorgente completa, il completamento non viene propagato fino al prossimo timing di campionamento.

### 4. Utilizzo Memoria

L'efficienza della memoria è buona perché mantiene internamente solo un ultimo valore.

## 💡 Differenza da sample

`sample` usa un altro Observable come trigger, mentre `sampleTime` usa intervalli di tempo fissi.

```ts
import { interval, fromEvent } from 'rxjs';
import { sample, sampleTime } from 'rxjs';

const source$ = interval(100);

// sampleTime: Intervallo tempo fisso (ogni 1 secondo)
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));

// sample: Usa altro Observable come trigger
const clicks$ = fromEvent(document, 'click');
source$.pipe(
  sample(clicks$)
).subscribe(val => console.log('sample:', val));
// Emetti ultimo valore a quel momento ogni volta che clicchi
```

| Operatore | Trigger | Caso d'Uso |
|:---|:---|:---|
| `sampleTime(ms)` | Intervallo tempo fisso | Campionamento periodico |
| `sample(notifier$)` | Altro Observable | Campionamento a timing dinamico |

## 📚 Operatori Correlati

- **[sample](https://rxjs.dev/api/operators/sample)** - Campiona usando altro Observable come trigger (documentazione ufficiale)
- **[throttleTime](/it/guide/operators/filtering/throttleTime)** - Ottieni primo valore all'inizio del periodo
- **[auditTime](/it/guide/operators/filtering/auditTime)** - Ottieni ultimo valore alla fine del periodo
- **[debounceTime](/it/guide/operators/filtering/debounceTime)** - Emetti valore dopo silenzio

## Riepilogo

L'operatore `sampleTime` campiona periodicamente l'ultimo valore a intervalli di tempo specificati.

- ✅ Ideale per acquisizione snapshot periodici
- ✅ Efficace per diradare stream ad alta frequenza
- ✅ Buona efficienza memoria (mantiene solo 1 ultimo valore)
- ✅ Ideale per dashboard e monitoraggio
- ⚠️ Non emette nulla se non c'è valore durante il periodo di campionamento
- ⚠️ Tempo di attesa fino al primo campionamento
- ⚠️ Il completamento si propaga al prossimo timing di campionamento
