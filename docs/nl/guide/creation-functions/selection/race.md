---
description: De race Creation Function realiseert een speciaal samenvoegingsproces dat alleen de eerste stream overneemt die een waarde uitgeeft uit meerdere Observables en de andere daarna negeert.
titleTemplate: ':title | RxJS'
---

# race - neem de stream die als eerste een waarde uitgeeft

`race` is een speciale samengevoegde Creation Function die gebruik maakt van **alleen de eerste Observable die een waarde uitgeeft** uit meerdere Observables,
en de andere Observables negeert.


## Basissyntaxis en gebruik

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Langzaam (5 seconden)'));
const fast$ = timer(2000).pipe(map(() => 'Snel (2 seconden)'));

race(slow$, fast$).subscribe(console.log);
// Output: Snel (2 seconden)
```

- Alleen de Observable die als eerste een waarde uitgeeft is de winnaar en gaat verder met volgende streams.
- Andere Observables worden genegeerd.

[🌐 RxJS Officiële Documentatie - `race`](https://rxjs.dev/api/index/function/race)


## Typische gebruikspatronen

- **Verwerk de vroegste van meerdere gebruikersacties (klikken, toetsaanslagen, scrollen)**
- **Neem de vroegste van meerdere triggers over, zoals handmatig versturen en automatisch opslaan**
- **Toon de eerst voltooide gegevens eerst onder meerdere gegevensverzamelingsprocessen**

## Praktische codevoorbeelden (met UI)

Simuleert een race om alleen de eerste uit te geven uit drie streams die op verschillende tijdstippen vuren.

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>race praktisch voorbeeld:</h3>';
document.body.appendChild(output);

// Different timing Observables
const slow$ = timer(5000).pipe(map(() => 'Langzaam (na 5 seconden)'));
const medium$ = timer(3000).pipe(map(() => 'Gemiddeld (na 3 seconden)'));
const fast$ = timer(2000).pipe(map(() => 'Snel (na 2 seconden)'));

const startTime = Date.now();

// Race start message
const waiting = document.createElement('div');
waiting.textContent = 'Race gestart... Wachten tot de eerste stream wordt uitgegeven.';
output.appendChild(waiting);

// Run race
race(slow$, medium$, fast$).subscribe(winner => {
  const endTime = Date.now();
  const elapsed = ((endTime - startTime) / 1000).toFixed(2);

  const result = document.createElement('div');
  result.innerHTML = `<strong>Winnaar:</strong> ${winner} (Verstreken tijd: ${elapsed} seconden)`;
  result.style.color = 'green';
  result.style.marginTop = '10px';
  output.appendChild(result);

  const explanation = document.createElement('div');
  explanation.textContent = '※ Alleen de eerste Observable die een waarde uitgeeft wordt geselecteerd.';
  explanation.style.marginTop = '5px';
  output.appendChild(explanation);
});
```

- Na 2 seconden wordt de eerste `fast$` uitgegeven en daarna wordt alleen `fast$` uitgevoerd.
- Andere `medium$` en `slow$` uitgifte worden genegeerd.


## Gerelateerde Operators

- **[raceWith](/nl/guide/operators/combination/raceWith)** - Pipeable Operator versie (gebruikt in pipeline)
- **[timeout](/nl/guide/operators/utility/timeout)** - Alleen timeout operator
- **[merge](/nl/guide/creation-functions/combination/merge)** - Voeg alle streams samen Creation Function
