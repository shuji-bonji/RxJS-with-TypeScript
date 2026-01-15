---
description: "bufferTime operator verzamelt waarden op regelmatige tijdsintervallen voor batchverwerking: Perfect voor het aggregeren van gebeurtenissen, logs en realtime datastreams"
titleTemplate: ':title | RxJS'
---

# bufferTime - Buffer op Tijdsinterval

De `bufferTime` operator voert **een array van waarden** uit op gespecificeerde tijdsintervallen.
Dit is nuttig wanneer u de stream wilt scheiden op een bepaalde hoeveelheid tijd en het wilt behandelen als een batchproces.

## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { bufferTime } from 'rxjs';

// Geef waarden elke 100ms uit
const source$ = interval(100);

source$.pipe(
  bufferTime(1000)
).subscribe(buffer => {
  console.log('Waarden verzameld in 1 seconde:', buffer);
});

// Output voorbeeld:
// Waarden verzameld in 1 seconde: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Waarden verzameld in 1 seconde: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
// ...
```

- Waarden uitgezonden in één seconde worden gegroepeerd in een array en op volgorde uitgevoerd.

[🌐 RxJS Officiële Documentatie - `bufferTime`](https://rxjs.dev/api/operators/bufferTime)

## 💡 Typische gebruikspatronen

- Verzend batches op regelmatige intervallen
- Verwerk gebruikersoperaties in batches (bijv. sleepoperaties)
- Verzamel data van sensoren en IoT-apparaten
- Uitdunnen en comprimeren van log- en trace-informatie

## 🧠 Praktisch codevoorbeeld (met UI)

Buffer klikgebeurtenissen voor 1 seconde en voer ze samen elke seconde uit.

```ts
import { fromEvent } from 'rxjs';
import { bufferTime } from 'rxjs';

// Maak uitvoergebied
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Klikgebeurtenisstroom
const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  bufferTime(1000)
).subscribe(clickArray => {
  const message = `Klikken in 1 seconde: ${clickArray.length}`;
  console.log(message);
  output.textContent = message;
});
```

- Het aantal klikken per seconde wordt als een samenvatting weergegeven.
- Het bufferproces stelt u in staat om opeenvolgende voorkomens van gebeurtenissen samen te beheren.
