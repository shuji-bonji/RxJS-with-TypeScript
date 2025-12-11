---
description: "Uitleg over het sequentieel combineren van meerdere Observables met de concat Creation Function. Omdat de eerste Observable voltooid moet zijn voordat de volgende start, is het geschikt voor stapsgewijze uitvoering, sequentiële UI-weergave en opeenvolgende API-aanroepen. Inclusief type-inferentie in TypeScript en praktische voorbeelden."
---

# concat - Streams sequentieel combineren

`concat` is een Creation Function die meerdere Observables **sequentieel uitvoert** in de opgegeven volgorde.
Nadat de vorige Observable `complete` is, begint de emissie van de volgende Observable.

## Basissyntaxis en gebruik

```ts
import { concat, of, delay } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));

concat(obs1$, obs2$).subscribe(console.log);
// Output: A → B → C → D
```

- Nadat alle emissies van `obs1$` voltooid zijn, begint de emissie van `obs2$`.
- Het belangrijkste punt is dat streams "op volgorde" stromen, niet gelijktijdig.

[🌐 RxJS Officiële Documentatie - `concat`](https://rxjs.dev/api/index/function/concat)


## Typische toepassingspatronen

- **Stapsgewijze verwerking**: Wanneer je na voltooiing van de vorige stap wilt doorgaan naar de volgende
- **API-requests met gegarandeerde volgorde**: Wanneer je een reeks asynchrone operaties op volgorde wilt uitvoeren
- **Controle van UI-events zoals animaties en notificaties** waar volgorde belangrijk is

## Praktisch codevoorbeeld (met UI)

Een voorbeeld van het **sequentieel weergeven** van een laadmelding en een datalist.

```ts
import { concat, of, timer } from 'rxjs';
import { map, take } from 'rxjs';

// Outputgebied aanmaken
const output = document.createElement('div');
output.innerHTML = '<h3>Praktisch voorbeeld van concat:</h3>';
document.body.appendChild(output);

// Laadstream
const loading$ = timer(0, 1000).pipe(
  map((count) => `⏳ Laden... (${count + 1}s)`),
  take(3) // Emit alleen gedurende 3 seconden
);

// Datalist stream
const data$ = of('🍎 Apple', '🍌 Banana', '🍇 Grape');

// concat gebruiken voor sequentiële weergave
concat(loading$, data$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- Eerst wordt de laadmelding 3 keer weergegeven,
- Daarna wordt de datalist op volgorde weergegeven.
- Met **concat** kun je eenvoudig een natuurlijke "gefaseerde weergave" realiseren.


## Gerelateerde operators

- **[concatWith](/nl/guide/operators/combination/concatWith)** - Pipeable Operator versie (voor gebruik in pipeline)
- **[concatMap](/nl/guide/operators/transformation/concatMap)** - Map en combineer elke waarde sequentieel
- **[merge](/nl/guide/creation-functions/combination/merge)** - Creation Function voor parallelle combinatie
