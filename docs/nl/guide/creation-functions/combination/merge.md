---
description: "De merge Creation Function subscribet gelijktijdig op meerdere Observables en integreert en output de respectievelijke waarden in real-time. Geschikt voor parallelle verwerking, integratie van meerdere event-bronnen en implementatie van real-time updates. Uitleg over typeveilige implementatie in TypeScript met praktische codevoorbeelden."
---

# merge - Meerdere streams gelijktijdig combineren

`merge` is een Creation Function die gelijktijdig subscribet op meerdere Observables,
en telkens wanneer een waarde van een Observable wordt uitgezonden, deze direct doorgeeft.

## Basissyntaxis en gebruik

```ts
import { merge, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Stream 1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Stream 2: ${val}`),
  take(2)
);

merge(source1$, source2$).subscribe(console.log);
// Voorbeeldoutput:
// Stream 1: 0
// Stream 2: 0
// Stream 1: 1
// Stream 1: 2
// Stream 2: 1
```

- Subscribet op alle Observables gelijktijdig en waarden stromen in **de volgorde waarin ze worden uitgezonden**.
- Er is geen garantie voor volgorde en het is **afhankelijk van de emissie-timing van elke Observable**.


[🌐 RxJS Officiële Documentatie - `merge`](https://rxjs.dev/api/index/function/merge)

## Typische toepassingspatronen

- **Integratie van meerdere asynchrone events** (bijv. gebruikersinvoer en backend-notificaties)
- **Aggregatie van meerdere datastreams tot één enkele stream**
- **Integratie van real-time updates en polling**, combinatie van parallelle informatiebronnen

## Praktisch codevoorbeeld (met UI)

Combineert en toont klik-events en timer-events in real-time.

```ts
import { merge, fromEvent, timer } from 'rxjs';
import { map } from 'rxjs';

// Outputgebied aanmaken
const output = document.createElement('div');
output.innerHTML = '<h3>Praktisch voorbeeld van merge:</h3>';
document.body.appendChild(output);

// Knop-element aanmaken
const button = document.createElement('button');
button.textContent = 'Klik om event te triggeren';
document.body.appendChild(button);

// Klik-stream
const click$ = fromEvent(button, 'click').pipe(
  map(() => '✅ Knopklik gedetecteerd')
);

// Timer-stream
const timer$ = timer(3000, 3000).pipe(
  map((val) => `⏰ Timer-event (${val})`)
);

// merge gebruiken voor weergave
merge(click$, timer$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- **Wanneer je op de knop klikt, wordt onmiddellijk een event gegenereerd**,
- **De timer genereert herhaaldelijk events om de 3 seconden**.
- Je kunt ervaren dat twee verschillende soorten Observables **in real-time kunnen worden geïntegreerd**.


## Gerelateerde operators

- **[mergeWith](/nl/guide/operators/combination/mergeWith)** - Pipeable Operator versie (voor gebruik in pipeline)
- **[mergeMap](/nl/guide/operators/transformation/mergeMap)** - Map en combineer elke waarde parallel
- **[concat](/nl/guide/creation-functions/combination/concat)** - Creation Function voor sequentiële combinatie
