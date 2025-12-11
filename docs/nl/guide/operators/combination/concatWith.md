---
description: concatWith is een RxJS-combinatie-operator die andere Observables sequentieel combineert na voltooiing van de originele Observable. Ideaal voor sequentiële verwerking in pipelines, follow-up verwerking na voltooiing, en gefaseerde data-loading waar je vervolgverwerking wilt toevoegen als extensie van de hoofdstream. Pipeable operator-versie voor handig gebruik in pipelines.
---

# concatWith - Streams sequentieel combineren in pipeline

De `concatWith`-operator combineert opgegeven andere Observables **sequentieel** nadat de originele Observable `complete` is.
Dit is de Pipeable Operator-versie van de Creation Function `concat`.

## 🔰 Basissyntax en gebruik

```ts
import { of, delay } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));
const obs3$ = of('E', 'F').pipe(delay(100));

obs1$
  .pipe(concatWith(obs2$, obs3$))
  .subscribe(console.log);

// Output: A → B → C → D → E → F
```

- `obs2$` start nadat `obs1$` compleet is, en `obs3$` start nadat `obs2$` compleet is.
- Kan gebruikt worden binnen `.pipe()`-chain, waardoor het gemakkelijk te combineren is met andere operators.

[🌐 RxJS Officiële Documentatie - `concatWith`](https://rxjs.dev/api/operators/concatWith)


## 💡 Typische toepassingspatronen

- **Sequentiële verwerking in pipeline**: Voeg extra data sequentieel toe aan getransformeerde stream
- **Follow-up verwerking na voltooiing**: Voeg cleanup of notificaties toe na hoofdverwerking
- **Gefaseerde data-loading**: Haal extra data sequentieel op na initiële data-ophaling


## 🧠 Praktisch codevoorbeeld (met UI)

Voorbeeld van het tonen van hoofdzoekresultaten gevolgd door aanbevolen items in volgorde.

```ts
import { of, delay } from 'rxjs';
import { concatWith, map } from 'rxjs';

// Maak output-gebied
const output = document.createElement('div');
output.innerHTML = '<h3>concatWith praktijkvoorbeeld:</h3>';
document.body.appendChild(output);

// Hoofdzoekresultaten
const searchResults$ = of('🔍 Zoekresultaat1', '🔍 Zoekresultaat2', '🔍 Zoekresultaat3').pipe(
  delay(500)
);

// Aanbevolen items 1
const recommendations1$ = of('💡 Aanbevolen product A', '💡 Aanbevolen product B').pipe(
  delay(300)
);

// Aanbevolen items 2
const recommendations2$ = of('⭐ Populair product X', '⭐ Populair product Y').pipe(
  delay(300)
);

// Combineer sequentieel en toon
searchResults$
  .pipe(
    concatWith(recommendations1$, recommendations2$),
    map((value, index) => `${index + 1}. ${value}`)
  )
  .subscribe((value) => {
    const item = document.createElement('div');
    item.textContent = value;
    output.appendChild(item);
  });
```

- Eerst worden zoekresultaten getoond,
- Daarna worden aanbevolen producten in volgorde getoond.
- Kan gecombineerd worden met andere operators zoals `map` binnen de pipeline.


## 🔄 Verschil met Creation Function `concat`

### Fundamentele verschillen

| | `concat` (Creation Function) | `concatWith` (Pipeable Operator) |
|:---|:---|:---|
| **Gebruikslocatie** | Gebruikt als onafhankelijke functie | Gebruikt binnen `.pipe()`-chain |
| **Notatie** | `concat(obs1$, obs2$, obs3$)` | `obs1$.pipe(concatWith(obs2$, obs3$))` |
| **Eerste stream** | Behandelt alle gelijkwaardig | Behandelt als hoofdstream |
| **Voordeel** | Eenvoudig en leesbaar | Gemakkelijk te combineren met andere operators |

### Concrete voorbeelden van keuze

**Voor alleen eenvoudige combinatie is Creation Function aanbevolen**

```ts
import { concat, of } from 'rxjs';

const part1$ = of('A', 'B');
const part2$ = of('C', 'D');
const part3$ = of('E', 'F');

// Eenvoudig en leesbaar
concat(part1$, part2$, part3$).subscribe(console.log);
// Output: A → B → C → D → E → F
```

**Bij noodzaak van tussentijdse transformatie is Pipeable Operator aanbevolen**

```ts
import { of } from 'rxjs';
import { concatWith, map, filter } from 'rxjs';

const userData$ = of({ name: 'Alice', age: 30 }, { name: 'Bob', age: 25 });
const additionalData$ = of({ name: 'Charlie', age: 35 });

// ❌ Creation Function versie - wordt omslachtig
import { concat } from 'rxjs';
concat(
  userData$.pipe(
    filter(user => user.age >= 30),
    map(user => user.name)
  ),
  additionalData$.pipe(map(user => user.name))
).subscribe(console.log);

// ✅ Pipeable Operator versie - compleet in één pipeline
userData$
  .pipe(
    filter(user => user.age >= 30),  // alleen 30+ jaar
    map(user => user.name),          // extract alleen naam
    concatWith(
      additionalData$.pipe(map(user => user.name))
    )
  )
  .subscribe(console.log);
// Output: Alice → Charlie
```

**Bij toevoegen vervolgverwerking aan hoofdstream**

```ts
import { fromEvent, of } from 'rxjs';
import { concatWith, take, mapTo } from 'rxjs';

// Maak knop en output-gebied
const button = document.createElement('button');
button.textContent = 'Klik 3 keer';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

const clicks$ = fromEvent(button, 'click');

// ✅ Pipeable Operator versie - natuurlijk als extensie van hoofdstream
clicks$
  .pipe(
    take(3),                          // neem eerste 3 kliks
    mapTo('Geklikt'),
    concatWith(of('Voltooid'))        // voeg bericht toe na voltooiing
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = message;
    output.appendChild(div);
  });

// Zelfde gedrag in Creation Function versie...
// ❌ Creation Function versie - moet hoofdstream apart schrijven
import { concat } from 'rxjs';
concat(
  clicks$.pipe(
    take(3),
    mapTo('Geklikt')
  ),
  of('Voltooid')
).subscribe(console.log);
```

### Samenvatting

- **`concat`**: Optimaal voor alleen eenvoudige combinatie van meerdere streams
- **`concatWith`**: Optimaal wanneer je vervolgverwerking wilt toevoegen terwijl je transformaties toepast op de hoofdstream


## ⚠️ Aandachtspunten

### Vertraging door wachten op voltooiing

De volgende Observable start niet totdat de originele Observable compleet is.

```ts
import { interval, of } from 'rxjs';
import { concatWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),              // laat voltooien na 3
  concatWith(of('Voltooid'))
).subscribe(console.log);
// Output: 0 → 1 → 2 → Voltooid
```

### Foutafhandeling

Als een fout optreedt in de vorige Observable, worden volgende Observables niet uitgevoerd.

```ts
import { throwError, of } from 'rxjs';
import { concatWith, catchError } from 'rxjs';

throwError(() => new Error('Fout opgetreden'))
  .pipe(
    catchError(err => of('Fout hersteld')),
    concatWith(of('Volgende verwerking'))
  )
  .subscribe(console.log);
// Output: Fout hersteld → Volgende verwerking
```


## 📚 Gerelateerde operators

- **[concat](/nl/guide/creation-functions/combination/concat)** - Creation Function-versie
- **[mergeWith](/nl/guide/operators/combination/mergeWith)** - Pipeable versie voor parallelle combinatie
- **[concatMap](/nl/guide/operators/transformation/concatMap)** - Map elke waarde sequentieel
