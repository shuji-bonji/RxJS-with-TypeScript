---
description: "L'operatore sampleTime è un operatore di filtraggio di RxJS che campiona periodicamente i valori più recenti del flusso a intervalli di tempo specificati. È ideale per scattare istantanee periodiche."
---

# sampleTime - ottiene periodicamente il valore più recente

L'operatore sampleTime periodicamente **campiona** il valore più recente dell'Observable di origine a **specificati intervalli di tempo** e lo restituisce.
Come un'istantanea periodica, recupera il valore più recente in quel momento.

## 🔰 Sintassi e uso di base

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Campioni secondo per secondo');
});
```

**Flusso di funzionamento**:.
1. il timer scatta periodicamente ogni 2 secondi
2. uscita se c'è un evento click recente in quel momento
3. se non c'è alcun valore durante il periodo di campionamento, non viene emesso alcun risultato.

> [!WARNING] Attenzione in codice di produzione

> L'esempio precedente omette la sottoscrizione di fromEvent per semplicità di spiegazione. Nel codice reale, usare takeUntil(destroy$)`, take(N)` o `Subscription.unsubscribe()` per gestire esplicitamente il ciclo di vita. Ulteriori informazioni: [Superare le difficoltà: la gestione del ciclo di vita] (/it/guide/ Superare le difficoltà/gestione del ciclo di vita.md)

[🌐 Documentazione ufficiale di RxJS - sampleTime](https://rxjs.dev/api/operators/sampleTime)

## 💡 Modelli tipici di utilizzo

- Acquisizione ricorrente dei dati dei sensori**: informazioni aggiornate sulla temperatura e sulla posizione ogni secondo.
- **Dashboard in tempo reale**: aggiornamenti regolari dello stato.
- **Monitoraggio delle prestazioni**: raccolta di metriche a intervalli regolari.
- **Elaborazione dei fotogrammi di gioco**: campionamento periodico per il controllo degli FPS

## 🧠 Esempio pratico di codice 1: campionamento periodico della posizione del mouse

Questo è un esempio di campionamento della posizione del mouse ogni secondo.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UICreazione
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Campionamento della posizione del mouse (1(ogni secondo)';
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
area.textContent = 'Spostamento del mouse all'interno di quest'area';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// Evento di spostamento del mouse
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // 1Campionamento ogni secondo
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>Campionamento #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    Posizione: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Massimo.10Visualizzazione di un massimo di
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- Se il mouse viene spostato continuamente, ogni secondo viene campionata solo l'ultima posizione corrente.
- Se il mouse non viene spostato per un secondo, non viene emesso nulla per quel periodo.

## 🎯 Esempio pratico di codice 2: cruscotto di dati in tempo reale

Questo esempio mostra come i dati del sensore possano essere campionati periodicamente e visualizzati su un cruscotto.

```ts
import { interval } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UICreazione
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Cruscotto di monitoraggio dei sensori';
container.appendChild(title);

const dashboard = document.createElement('div');
dashboard.style.display = 'grid';
dashboard.style.gridTemplateColumns = '1fr 1fr';
dashboard.style.gap = '10px';
dashboard.style.marginTop = '10px';
container.appendChild(dashboard);

// Creazione della scheda del cruscotto
function createCard(label: string, unit: string) {
  const card = document.createElement('div');
  card.style.padding = '20px';
  card.style.border = '2px solid #2196F3';
  card.style.borderRadius = '8px';
  card.style.backgroundColor = '#E3F2FD';

  const labelDiv = document.createElement('div');
  labelDiv.textContent = label;
  labelDiv.style.fontSize = '14px';
  labelDiv.style.color = '#666';
  card.appendChild(labelDiv);

  const valueDiv = document.createElement('div');
  valueDiv.style.fontSize = '32px';
  valueDiv.style.fontWeight = 'bold';
  valueDiv.style.marginTop = '10px';
  valueDiv.textContent = '--';
  card.appendChild(valueDiv);

  const unitDiv = document.createElement('div');
  unitDiv.textContent = unit;
  unitDiv.style.fontSize = '14px';
  unitDiv.style.color = '#666';
  card.appendChild(unitDiv);

  dashboard.appendChild(card);
  return valueDiv;
}

const tempValue = createCard('Temperatura', '°C');
const humidityValue = createCard('Umidità', '%');
const pressureValue = createCard('Pressione barometrica', 'hPa');
const lightValue = createCard('Illuminamento', 'lux');

// Flusso di dati del sensore (100msAggiornato ogni)
const sensorData$ = interval(100).pipe(
  map(() => ({
    temperature: (20 + Math.random() * 10).toFixed(1),
    humidity: (40 + Math.random() * 40).toFixed(1),
    pressure: (1000 + Math.random() * 30).toFixed(1),
    light: Math.floor(Math.random() * 1000)
  }))
);

// 2Campionamento e aggiornamento del cruscotto ogni secondo
sensorData$.pipe(
  sampleTime(2000)
).subscribe(data => {
  tempValue.textContent = data.temperature;
  humidityValue.textContent = data.humidity;
  pressureValue.textContent = data.pressure;
  lightValue.textContent = data.light.toString();

  // Effetto animazione
  [tempValue, humidityValue, pressureValue, lightValue].forEach(elem => {
    elem.style.color = '#2196F3';
    setTimeout(() => {
      elem.style.color = 'black';
    }, 500);
  });
});
```

- I dati del sensore vengono aggiornati ogni 100 ms, mentre il cruscotto viene aggiornato con i valori campionati ogni 2 secondi.
- Le prestazioni possono essere ottimizzate visualizzando flussi di dati ad alta frequenza a intervalli appropriati.

## 🆚 Confronto con operatori simili

### sampleTime vs throttleTime vs auditTime

```ts
import { interval } from 'rxjs';
import { sampleTime, throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, ...

// sampleTime: 1Campionamento dell'ultimo valore in quel momento ogni secondo
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));
// Esempi di output: 2, 5, 8(1Istantanea ogni secondo)

// throttleTime: Dopo l'emissione del primo valore,1Ignorato per 2 secondi dopo l'emissione del primo valore
source$.pipe(
  throttleTime(1000)
).subscribe(val => console.log('throttleTime:', val));
// Esempi di output: 0, 3, 6, 9(primo valore di ogni periodo)

// auditTime: Emissione dell'ultimo valore del periodo1secondi dopo il primo valore, viene emesso l'ultimo valore del periodo
source$.pipe(
  auditTime(1000)
).subscribe(val => console.log('auditTime:', val));
// Esempi di output: 2, 5, 8(ultimo valore di ogni periodo)
```

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Campioni secondo per secondo');
});
```

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Campione al secondo');
});
```

**differenze visive**:.

```
Ingresso: --|1|2|3|---|4|5|6|---|7|8|9|
      0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (Campionamento periodico)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (Ignorato durante il periodo fino all'inizio)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (Ultimo valore alla fine del periodo)
```

## ⚠️ Note.

### 1. nessun valore durante il periodo di campionamento

Se non ci sono nuovi valori durante il periodo di campionamento, non viene prodotta alcuna uscita.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Campioni prelevati');
});
// 2Durante i secondi1Nessuna uscita se non vengono effettuati clic
```

### 2. Attendere il primo tempo di campionamento

Il parametro sampleTime non emetterà nulla finché non sarà trascorso il tempo specificato.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

interval(100).pipe(
  sampleTime(1000)
).subscribe(console.log);
// Il primo valore è1secondi dopo l'emissione del primo valore
```

### 3. completionTime

Quando una sorgente viene completata, il completamento non viene propagato fino alla successiva temporizzazione del campione.

```ts
import { of } from 'rxjs';
import { sampleTime, delay } from 'rxjs';

of(1, 2, 3).pipe(
  delay(100),
  sampleTime(1000)
).subscribe({
  next: console.log,
  complete: () => console.log('Completato')
});
// 1Secondi dopo: 3
// 1Secondi dopo: Completato
```

### 4. utilizzo della memoria

L'efficienza della memoria è buona, poiché viene mantenuto internamente solo un ultimo valore.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

// Flusso ad alta frequenza (10msal secondo)
interval(10).pipe(
  sampleTime(1000) // 1Campionamento ogni secondo
).subscribe(console.log);
// La memoria conserva solo i valori più recenti1Vengono mantenuti in memoria solo i due valori più recenti.
```

## 💡 Differenze con il sample

sample utilizza un altro Observable come trigger, mentre sampleTime utilizza un intervallo di tempo fisso.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Campioni secondo per secondo');
});

```

## 📚 Operatori correlati.

- **[sample](https://rxjs.dev/api/operators/sample)** - Campionamento di un altro Observable come trigger (documentazione ufficiale).
- **[throttleTime](. /throttleTime)** - Ottiene il primo valore all'inizio del periodo.
- **[auditTime](. /auditTime)** - ottiene l'ultimo valore alla fine del periodo.
- **[debounceTime](. /debounceTime)** - emette il valore dopo la quiescenza.

## Riepilogo.

L'operatore sampleTime campiona periodicamente il valore più recente nell'intervallo di tempo specificato.

- Ideale per ottenere istantanee periodiche.
- ✅ Utile per sfoltire i flussi ad alta frequenza
- ✅ Efficiente dal punto di vista della memoria (viene conservato solo l'ultimo valore)
- ✅ Ideale per dashboard e monitoraggio
- ⚠️ Se non sono disponibili valori durante il periodo di campionamento, non viene emesso nulla.
- ⚠️ C'è un periodo di attesa fino al primo campione
- ⚠️ Il completamento viene propagato alla successiva tempistica di campionamento
