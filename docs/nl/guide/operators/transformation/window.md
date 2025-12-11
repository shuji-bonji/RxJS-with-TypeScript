---
description: window is een RxJS operator die een bron Observable splitst in geneste Observables op het moment dat een andere Observable waarden uitzendt, ideaal voor geavanceerde event-gedreven streamverwerking.
titleTemplate: ':title | RxJS'
---

# window - Splits Observable op timing van andere Observable

De `window` operator groepeert de waarden van een bron Observable **totdat een andere Observable waarden uitzendt** en geeft die groep uit als een **nieuwe Observable**.
Terwijl `buffer` een array retourneert, retourneert `window` een **Observable&lt;T&gt;**, waardoor verdere operators op elk venster kunnen worden toegepast.

## 🔰 Basissyntax en gebruik

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

// Geef waarden elke 100ms uit
const source$ = interval(100);

// Gebruik klikgebeurtenis als trigger
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  window(clicks$),
  mergeAll() // Plat elke venster
).subscribe(value => {
  console.log('Waarde in venster:', value);
});

// Een nieuw venster start met elke klik
```

- Elke keer dat `clicks$` een waarde uitzendt, wordt een nieuw venster (Observable) gemaakt.
- Elk venster kan worden behandeld als een onafhankelijke Observable.

[🌐 RxJS Officiële Documentatie - `window`](https://rxjs.dev/api/operators/window)

## 💡 Typische gebruikspatronen

- Event-gedreven stream partitionering
- Verschillende verwerking toepassen op elk venster
- Data groepering met dynamische afbakening
- Aggregaatverwerking voor elk venster

## 🔍 Verschil met buffer

| Operator | Uitvoer | Gebruiksscenario |
|:---|:---|:---|
| `buffer` | **Array (T[])** | Gegroepeerde waarden samen verwerken |
| `window` | **Observable&lt;T&gt;** | Verschillende streamverwerking voor elke groep |

```ts
import { interval, timer } from 'rxjs';
import { buffer, window, mergeAll } from 'rxjs';

const source$ = interval(100);
const trigger$ = timer(1000, 1000);

// buffer - Uitvoer als array
source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Uitvoer: Buffer (array): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// window - Uitvoer als Observable
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  console.log('Venster (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Waarde in venster:', value);
  });
});
```

## 🧠 Praktisch codevoorbeeld 1: Tellen per venster

Dit voorbeeld triggert op knopklik en telt het aantal gebeurtenissen tot dat punt.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, map, mergeAll, scan } from 'rxjs';

// Maak knop
const button = document.createElement('button');
button.textContent = 'Afbakenen venster';
document.body.appendChild(button);

// Uitvoergebied
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Geef waarden elke 100ms uit
const source$ = interval(100);

// Trigger op knopklik
const clicks$ = fromEvent(button, 'click');

let windowCount = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const currentWindow = ++windowCount;
    console.log(`Venster ${currentWindow} gestart`);

    // Tel waarden in elk venster
    return window$.pipe(
      scan((count) => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  output.textContent = `Huidig venster: ${windowCount}, Telling: ${count}`;
});
```

- Elke keer dat een knop wordt geklikt, wordt een nieuw venster gemaakt.
- Het aantal waarden in elk venster wordt realtime geteld.

## 🎯 Praktisch codevoorbeeld 2: Verschillende verwerking voor elk venster

Dit is een geavanceerd voorbeeld dat verschillende verwerking toepast op elk venster.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, take, mergeAll, map } from 'rxjs';

const source$ = interval(200);
const clicks$ = fromEvent(document, 'click');

let windowNumber = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Even vensters: Krijg alleen eerste 3 items
      console.log(`Venster ${current}: Krijg eerste 3 items`);
      return window$.pipe(take(3));
    } else {
      // Oneven vensters: Krijg alles
      console.log(`Venster ${current}: Krijg alles`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`Waarde: ${value} (Venster ${windowNumber})`);
});
```

- U kunt conditioneel verschillende verwerking toepassen voor elk venster.
- Elk venster is een onafhankelijke Observable, dus u kunt vrij operators combineren.

## 🎯 Praktisch voorbeeld: Controle met meerdere triggers

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { window, mergeAll, scan, map } from 'rxjs';

const source$ = interval(100);

// Meerdere triggers: klik of 3 seconden verstreken
const clicks$ = fromEvent(document, 'click');
const threeSeconds$ = timer(3000, 3000);
const trigger$ = merge(clicks$, threeSeconds$);

source$.pipe(
  window(trigger$),
  map((window$, index) => {
    console.log(`Venster ${index + 1} gestart`);

    // Bereken som voor elk venster
    return window$.pipe(
      scan((sum, value) => sum + value, 0)
    );
  }),
  mergeAll()
).subscribe(sum => {
  console.log('Huidige som:', sum);
});
```

## ⚠️ Opmerkingen

### 1. Venster abonnementbeheer

Elk venster is een onafhankelijke Observable, dus er moet expliciet op worden geabonneerd.

```ts
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  // Waarden stromen niet tenzij u abonneert op het venster zelf
  window$.subscribe(value => {
    console.log('Waarde:', value);
  });
});
```

Of gebruik `mergeAll()`, `concatAll()`, `switchAll()`, etc. om te plat te maken.

```ts
source$.pipe(
  window(trigger$),
  mergeAll() // Voeg alle vensters samen
).subscribe(value => {
  console.log('Waarde:', value);
});
```

### 2. Pas op voor geheugenlekken

**Probleem**: Als de trigger Observable geen waarden uitzendt, blijft het eerste venster voor altijd open en accumuleren waarden oneindig.

#### ❌ Slecht voorbeeld: Trigger treedt niet op

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100); // Blijft waarden uitzenden elke 100ms

// Knop bestaat niet, of gebruiker klikt niet
const button = document.querySelector('#start-button'); // Mogelijk null
const clicks$ = fromEvent(button, 'click'); // Fout of vuurt nooit

source$.pipe(
  window(clicks$), // Eerste venster sluit niet als clicks$ niet vuurt
  mergeAll()
).subscribe();

// Problemen:
// - Als clicks$ niet uitzendt, blijft eerste venster open
// - source$ waarden (0, 1, 2, 3...) blijven accumuleren in geheugen
// - Veroorzaakt geheugenlek
```

#### ✅ Goed voorbeeld 1: Stel timeout in

Stel een timeout in om te voorkomen dat het eerste venster te lang open blijft.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = button ? fromEvent(button, 'click') : interval(0); // fallback

// Sluit venster bij klik of na 5 seconden, wat het eerst komt
const autoClose$ = timer(5000); // Zendt uit na 5 seconden
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // Venster sluit altijd binnen 5 seconden
  mergeAll()
).subscribe();
```

#### ✅ Goed voorbeeld 2: Sluit vensters periodiek

Sluit vensters periodiek zelfs zonder klikken.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = fromEvent(button, 'click');

// Sluit venster bij klik of elke 3 seconden
const autoClose$ = timer(3000, 3000); // Na eerste 3 seconden, dan elke 3 seconden
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // Venster sluit elke 3 seconden zelfs zonder klikken
  mergeAll()
).subscribe();

// Resultaat:
// - Vensters sluiten automatisch elke 3 seconden zelfs zonder gebruikersklikken
// - Voorkomt oneindige waarde-accumulatie in geheugen
```

### 3. Venster overlap

Standaard overlappen vensters niet (volgend venster start nadat vorige sluit).
Als overlap nodig is, gebruik `windowToggle` of `windowWhen`.

## 🆚 Vergelijking van window operators

| Operator | Timing van afbakening | Gebruiksscenario |
|:---|:---|:---|
| `window` | Andere Observable zendt uit | Event-gedreven partitionering |
| `windowTime` | Vast tijdsinterval | Tijd-gebaseerde partitionering |
| `windowCount` | Vast aantal | Aantal-gebaseerde partitionering |
| `windowToggle` | Start en eind Observables | Dynamische start/eind controle |
| `windowWhen` | Dynamische sluitconditie | Verschillende eindconditie per venster |

## 📚 Gerelateerde operators

- [`buffer`](/nl/guide/operators/transformation/buffer) - Verzamel waarden als array (array versie van window)
- [`windowTime`](/nl/guide/operators/transformation/windowTime) - Tijd-gebaseerde venster partitionering
- [`windowCount`](/nl/guide/operators/transformation/windowCount) - Aantal-gebaseerde venster partitionering
- [`windowToggle`](/nl/guide/operators/transformation/windowToggle) - Venstercontrole met start en eind Observables
- [`windowWhen`](/nl/guide/operators/transformation/windowWhen) - Venster partitionering met dynamische sluitconditie
- [`groupBy`](/nl/guide/operators/transformation/groupBy) - Groepeer Observables op sleutel

## Samenvatting

De `window` operator is een krachtig hulpmiddel dat streams splitst getriggerd door een externe Observable en elke groep kan verwerken als een onafhankelijke Observable.

- ✅ Kan verschillende verwerking toepassen op elk venster
- ✅ Flexibele event-gedreven controle
- ✅ Ondersteunt geavanceerde streamoperaties
- ⚠️ Abonnementbeheer vereist
- ⚠️ Pas op voor geheugenlekken
