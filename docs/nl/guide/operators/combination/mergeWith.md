---
description: "mergeWith is een Pipeable Operator die de originele Observable en andere Observables gelijktijdig subscribed en parallel combineert. Kan gebruikt worden voor het integreren van meerdere event-bronnen voor real-time verwerking. Uitleg over verschillen met merge() en type-veilige implementatie in TypeScript."
---

# mergeWith - Meerdere streams gelijktijdig combineren in pipeline

De `mergeWith`-operator **subscribed gelijktijdig** op de originele Observable en opgegeven andere Observables,
en integreert real-time waarden die van elk uitgegeven worden.
Dit is de Pipeable Operator-versie van de Creation Function `merge`.

## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Stream1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Stream2: ${val}`),
  take(2)
);

source1$
  .pipe(mergeWith(source2$))
  .subscribe(console.log);

// Voorbeeldoutput:
// Stream1: 0
// Stream2: 0
// Stream1: 1
// Stream1: 2
// Stream2: 1
```

- Subscribed gelijktijdig op alle Observables en waarden stromen in **uitgegeven volgorde**.
- Volgorde is niet gegarandeerd, **afhankelijk van uitgifte-timing van elke Observable**.

[🌐 RxJS Officiële Documentatie - `mergeWith`](https://rxjs.dev/api/operators/mergeWith)


## 💡 Typische toepassingspatronen

- **Integreren van meerdere event-bronnen**: Integratie van gebruikersacties en automatische updates
- **Combineren van parallelle data-fetches**: Aggregeren van responses van meerdere APIs in enkele stream
- **Mergen van real-time updates**: Integreren van WebSocket en polling


## 🧠 Praktisch codevoorbeeld (met UI)

Voorbeeld van het integreren van gebruikersklikevents en automatische update-timer voor notificaties.

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

// Maak output-gebied
const output = document.createElement('div');
output.innerHTML = '<h3>mergeWith praktijkvoorbeeld:</h3>';
document.body.appendChild(output);

// Maak knop
const button = document.createElement('button');
button.textContent = 'Handmatige update';
document.body.appendChild(button);

// Klikstream
const manualUpdate$ = fromEvent(button, 'click').pipe(
  map(() => '👆 Handmatige update uitgevoerd')
);

// Automatische update-timer (elke 5 seconden)
const autoUpdate$ = interval(5000).pipe(
  map(val => `🔄 Automatische update #${val + 1}`),
  take(3)
);

// Integreer beide en toon
manualUpdate$
  .pipe(mergeWith(autoUpdate$))
  .subscribe((value) => {
    const timestamp = new Date().toLocaleTimeString();
    const item = document.createElement('div');
    item.textContent = `[${timestamp}] ${value}`;
    output.appendChild(item);
  });
```

- Klik op knop toont onmiddellijk handmatige update,
- Automatische update voert ook parallel uit elke 5 seconden.
- Beide events worden real-time geïntegreerd.


## 🔄 Verschil met Creation Function `merge`

### Fundamentele verschillen

| | `merge` (Creation Function) | `mergeWith` (Pipeable Operator) |
|:---|:---|:---|
| **Gebruikslocatie** | Gebruikt als onafhankelijke functie | Gebruikt binnen `.pipe()`-chain |
| **Notatie** | `merge(obs1$, obs2$, obs3$)` | `obs1$.pipe(mergeWith(obs2$, obs3$))` |
| **Eerste stream** | Behandelt alle gelijkwaardig | Behandelt als hoofdstream |
| **Voordeel** | Eenvoudig en leesbaar | Gemakkelijk te combineren met andere operators |

### Concrete voorbeelden van keuze

**Voor alleen eenvoudige merge is Creation Function aanbevolen**

```ts
import { merge, fromEvent } from 'rxjs';
import { map } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(map(() => 'Klik'));
const moves$ = fromEvent(document, 'mousemove').pipe(map(() => 'Muisbeweging'));
const keypress$ = fromEvent(document, 'keypress').pipe(map(() => 'Toetsaanslag'));

// Eenvoudig en leesbaar
merge(clicks$, moves$, keypress$).subscribe(console.log);
// Output: toont in volgorde van welk event ook optreedt
```

**Bij toevoegen transformatieverwerking aan hoofdstream is Pipeable Operator aanbevolen**

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, filter, throttleTime } from 'rxjs';

const userClicks$ = fromEvent(document, 'click');
const autoRefresh$ = interval(30000); // elke 30 seconden

// ✅ Pipeable Operator versie - compleet in één pipeline
userClicks$
  .pipe(
    throttleTime(1000),           // voorkom snelle herhaalde kliks
    map(() => ({ source: 'user', timestamp: Date.now() })),
    mergeWith(
      autoRefresh$.pipe(
        map(() => ({ source: 'auto', timestamp: Date.now() }))
      )
    ),
    filter(event => event.timestamp > Date.now() - 60000)  // alleen binnen 1 minuut
  )
  .subscribe(event => {
    console.log(`${event.source}update: ${new Date(event.timestamp).toLocaleTimeString()}`);
  });

// ❌ Creation Function versie - wordt omslachtig
import { merge } from 'rxjs';
merge(
  userClicks$.pipe(
    throttleTime(1000),
    map(() => ({ source: 'user', timestamp: Date.now() }))
  ),
  autoRefresh$.pipe(
    map(() => ({ source: 'auto', timestamp: Date.now() }))
  )
).pipe(
  filter(event => event.timestamp > Date.now() - 60000)
).subscribe(event => {
  console.log(`${event.source}update: ${new Date(event.timestamp).toLocaleTimeString()}`);
});
```

**Bij integreren van meerdere databronnen**

```ts
import { fromEvent, timer } from 'rxjs';
import { mergeWith, map, startWith } from 'rxjs';

// Maak knop
const saveButton = document.createElement('button');
saveButton.textContent = 'Opslaan';
document.body.appendChild(saveButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Hoofdstream: gebruikersopslagactie
const manualSave$ = fromEvent(saveButton, 'click').pipe(
  map(() => '💾 Handmatig opslaan')
);

// ✅ Pipeable Operator versie - voeg automatisch opslaan toe aan hoofdstream
manualSave$
  .pipe(
    startWith('📝 Begin bewerking'),
    mergeWith(
      timer(10000, 10000).pipe(map(() => '⏰ Automatisch opslaan'))  // elke 10 seconden automatisch opslaan
    )
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
    output.appendChild(div);
  });
```

### Samenvatting

- **`merge`**: Optimaal voor alleen gelijkwaardig integreren van meerdere streams
- **`mergeWith`**: Optimaal wanneer je andere streams wilt integreren terwijl je transformaties toepast op de hoofdstream


## ⚠️ Aandachtspunten

### Voltooiingstiming

De gecombineerde stream voltooit niet totdat alle Observables voltooien.

```ts
import { of, interval, NEVER } from 'rxjs';
import { mergeWith, take } from 'rxjs';

of(1, 2, 3).pipe(
  mergeWith(
    interval(1000).pipe(take(2)),
    // NEVER  // ← toevoegen van dit voltooit nooit
  )
).subscribe({
  next: console.log,
  complete: () => console.log('✅ Voltooid')
});
// Output: 1 → 2 → 3 → 0 → 1 → ✅ Voltooid
```

### Controle van parallelle uitvoering

Standaard voert alle streams gelijktijdig uit, maar kan gecontroleerd worden met combinatie van `mergeMap`.

```ts
import { from, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

from([1, 2, 3, 4, 5]).pipe(
  mergeMap(
    val => of(val).pipe(delay(1000)),
    2  // maximaal 2 parallelle uitvoeringen
  )
).subscribe(console.log);
```

### Foutafhandeling

Als een fout optreedt in één van de Observables, eindigt het geheel met een fout.

```ts
import { throwError, interval } from 'rxjs';
import { mergeWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  mergeWith(
    throwError(() => new Error('Fout opgetreden')).pipe(
      catchError(err => of('Fout hersteld'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Fout:', err.message)
});
// Output: 0 → Fout hersteld → 1
```


## 📚 Gerelateerde operators

- **[merge](/nl/guide/creation-functions/combination/merge)** - Creation Function-versie
- **[concatWith](/nl/guide/operators/combination/concatWith)** - Pipeable versie voor sequentiële combinatie
- **[mergeMap](/nl/guide/operators/transformation/mergeMap)** - Map elke waarde parallel
