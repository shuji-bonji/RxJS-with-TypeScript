---
description: buffer is een RxJS operator die een array van geaccumuleerde waarden uitvoert op het moment dat een andere Observable een waarde uitzendt, waardoor het ideaal is voor event-gedreven batchverwerking.
titleTemplate: ':title | RxJS'
---

# buffer - Verzamelen op Gebeurtenis

De `buffer` operator accumuleert de waarden van een bron Observable **totdat** een andere Observable een waarde uitzendt, en voert vervolgens de geaccumuleerde waarden als een **array** uit op die timing.
Dit is nuttig wanneer u buffering wilt controleren volgens externe gebeurtenissen of signalen, in plaats van op tijd of aantal items.

## 🔰 Basissyntax en gebruik

```ts
import { interval, fromEvent } from 'rxjs';
import { buffer } from 'rxjs';

// Geef waarden elke 100ms uit
const source$ = interval(100);

// Gebruik klikgebeurtenis als trigger
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  buffer(clicks$)
).subscribe(bufferedValues => {
  console.log('Waarden geaccumuleerd tot klik:', bufferedValues);
});

// Output voorbeeld (uitvoer bij elke klik):
// Waarden geaccumuleerd tot klik: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
// Waarden geaccumuleerd tot klik: [11, 12, 13, 14, 15, 16, 17]
// ...
```

- Elke keer dat `clicks$` een waarde uitzendt, worden de tot dan toe geaccumuleerde waarden als een array uitgevoerd.
- Het kenmerk is dat bufferafscheiding kan worden gecontroleerd door een externe Observable.

[🌐 RxJS Officiële Documentatie - `buffer`](https://rxjs.dev/api/operators/buffer)

## 💡 Typische gebruikspatronen

- Batchverwerking getriggerd door gebruikersacties
- Dataverzameling en verzending op basis van externe signalen
- Gebeurtenisgroepering met dynamische afbakening
- Batch verzending wanneer WebSocket of API-verbinding is opgezet

## 🔍 Verschil met bufferTime / bufferCount

| Operator | Timing van afscheiding | Gebruik |
|:---|:---|:---|
| `buffer` | **Andere Observable zendt uit** | Event-gedreven controle |
| `bufferTime` | **Vast tijdsinterval** | Tijd-gebaseerde batchverwerking |
| `bufferCount` | **Vast aantal** | Aantal-gebaseerde batchverwerking |

```ts
import { interval, timer } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);
// Trigger elke 1 seconde
const trigger$ = timer(1000, 1000);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Waarden elke seconde:', values);
});

// Output:
// Waarden elke seconde: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Waarden elke seconde: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
```

## 🧠 Praktisch codevoorbeeld (met UI)

Dit is een voorbeeld van het triggeren van een knopklik en het samen registreren van alle muisbewegingsgebeurtenissen tot dat punt.

```ts
import { fromEvent } from 'rxjs';
import { map, buffer } from 'rxjs';

// Maak knop en uitvoergebied
const button = document.createElement('button');
button.textContent = 'Registreer muisbeweging';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Muisbewegingsgebeurtenis
const mouseMoves$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
);

// Trigger bij knopklik
const clicks$ = fromEvent(button, 'click');

mouseMoves$.pipe(
  buffer(clicks$)
).subscribe(positions => {
  const message = `Gedetecteerde gebeurtenissen: ${positions.length} items`;
  console.log(message);
  console.log('Coördinaatdata:', positions.slice(0, 5)); // Toon alleen eerste 5
  output.textContent = message;
});
```

- Alle muisbewegingen tot de knopklik worden opgeslagen in een buffer.
- Aangezien de gebeurtenissen samen worden verwerkt op het moment van de klik, is batchverwerking op willekeurige timing mogelijk.

## 🎯 Geavanceerd voorbeeld met meerdere triggers

Flexibelere controle is mogelijk door meerdere trigger Observables te combineren.

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { buffer, mapTo } from 'rxjs';

const source$ = interval(100);

// Meerdere triggers: klik of 5 seconden verstreken
const clicks$ = fromEvent(document, 'click').pipe(mapTo('klik'));
const fiveSeconds$ = timer(5000, 5000).pipe(mapTo('timer'));
const trigger$ = merge(clicks$, fiveSeconds$);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log(`Buffer uitvoer (${values.length} items):`, values);
});
```

## ⚠️ Opmerkingen

### Pas op voor geheugenlekken

Omdat `buffer` waarden blijft accumuleren tot de volgende trigger, kan het excessief geheugen verbruiken als er lange tijd geen trigger optreedt.

```ts
// Slecht voorbeeld: Trigger treedt mogelijk niet op
const neverTrigger$ = fromEvent(document.querySelector('.non-existent'), 'click');

source$.pipe(
  buffer(neverTrigger$) // Trigger treedt nooit op, buffer accumuleert oneindig
).subscribe();
```

**Tegenmaatregelen**:
- Beperk de maximale buffergrootte in combinatie met `bufferTime` en `bufferCount`
- Voeg timeout-afhandeling toe

```ts
import { interval, fromEvent, timer, race } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);

// Meerdere triggers: klik of 5 seconden verstreken
const clicks$ = fromEvent(document, 'click');
const timeout$ = timer(10000); // Timeout na maximaal 10 seconden

source$.pipe(
  buffer(race(clicks$, timeout$)) // Uitvoer op wat het eerst komt
).subscribe(values => {
  console.log('Buffer:', values);
});
```

## 📚 Gerelateerde operators

- [`bufferTime`](/nl/guide/operators/transformation/bufferTime) - Tijd-gebaseerde buffering
- [`bufferCount`](/nl/guide/operators/transformation/bufferCount) - Aantal-gebaseerde buffering
- [`bufferToggle`](https://rxjs.dev/api/operators/bufferToggle) - Buffercontrole met start en eind Observable
- [`bufferWhen`](https://rxjs.dev/api/operators/bufferWhen) - Buffering met dynamische sluitvoorwaarden
- [`window`](/nl/guide/operators/transformation/windowTime) - Retourneert Observable in plaats van buffer

## Samenvatting

De `buffer` operator is een krachtig hulpmiddel voor het verwerken van een batch waarden getriggerd door een externe Observable. Het maakt **event-gedreven** batchverwerking mogelijk, in plaats van op tijd of aantal items. Echter, pas op voor geheugenlekken wanneer triggers niet optreden.
