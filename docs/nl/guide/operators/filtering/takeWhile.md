---
description: "takeWhile is een RxJS filteroperator die waarden blijft nemen terwijl de gespecificeerde voorwaarde waar is en de stream voltooit wanneer de voorwaarde onwaar wordt. Het is ideaal voor situaties waar u een stream wilt controleren met dynamische voorwaarden, zoals data-acquisitie tot een drempelwaarde, prioriteit-gebaseerde verwerking, paginering, etc. De inclusive optie stelt u in staat om waarden op te nemen waarvoor de voorwaarde onwaar wordt."
---

# takeWhile - Neem waarden terwijl voorwaarde voldaan is

De `takeWhile` operator blijft waarden nemen **terwijl de gespecificeerde voorwaarde waar is**, en voltooit de stream wanneer de voorwaarde `false` wordt.


## 🔰 Basissyntax en gebruik

```ts
import { interval } from 'rxjs';
import { takeWhile } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('Voltooid')
});
// Output: 0, 1, 2, 3, 4, Voltooid
```

**Werkingsstroom**:
1. 0 wordt uitgegeven → `0 < 5` is `true` → Uitvoer
2. 1 wordt uitgegeven → `1 < 5` is `true` → Uitvoer
3. 2 wordt uitgegeven → `2 < 5` is `true` → Uitvoer
4. 3 wordt uitgegeven → `3 < 5` is `true` → Uitvoer
5. 4 wordt uitgegeven → `4 < 5` is `true` → Uitvoer
6. 5 wordt uitgegeven → `5 < 5` is `false` → Voltooien (5 wordt niet uitgevoerd)

[🌐 RxJS Officiële Documentatie - `takeWhile`](https://rxjs.dev/api/operators/takeWhile)


## 🆚 Contrast met take

`take` en `takeWhile` hebben verschillende acquisitievoorwaarden.

```ts
import { interval } from 'rxjs';
import { take, takeWhile } from 'rxjs';

const source$ = interval(1000);

// take: Controle op aantal
source$.pipe(
  take(5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4

// takeWhile: Controle op voorwaarde
source$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4
```

| Operator | Controlemethode | Voltooiingsvoorwaarde | Laatste waarde |
|---|---|---|---|
| `take(n)` | Aantal | Na n waarden | Inclusief nde waarde |
| `takeWhile(predicate)` | Voorwaarde functie | Wanneer voorwaarde `false` wordt | Inclusief geen waarde die `false` werd* |

\* Standaard wordt de waarde die `false` werd niet uitgevoerd, maar kan worden opgenomen met de `inclusive: true` optie


## 🎯 inclusive optie

Als u de waarde waarvoor de voorwaarde `false` werd wilt opnemen, specificeer `inclusive: true`.

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

const numbers$ = range(0, 10);

// Standaard (inclusive: false)
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4

// inclusive: true
numbers$.pipe(
  takeWhile(n => n < 5, true)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5 (inclusief 5 die voorwaarde onwaar maakte)
```


## 💡 Typische gebruikspatronen

1. **Data-acquisitie tot drempelwaarde**
   ```ts
   import { interval } from 'rxjs';
   import { takeWhile, map } from 'rxjs';

   // Temperatuursensor simulatie
   const temperature$ = interval(100).pipe(
     map(() => 20 + Math.random() * 15)
   );

   // Registreer alleen terwijl onder 30 graden
   temperature$.pipe(
     takeWhile(temp => temp < 30)
   ).subscribe({
     next: temp => console.log(`Temperatuur: ${temp.toFixed(1)}°C`),
     complete: () => console.log('Waarschuwing: Temperatuur overschreed 30 graden!')
   });
   ```

2. **Voorwaardelijke arrayverwerking**
   ```ts
   import { from } from 'rxjs';
   import { takeWhile } from 'rxjs';

   interface Task {
     id: number;
     priority: 'high' | 'medium' | 'low';
     completed: boolean;
   }

   const tasks$ = from([
     { id: 1, priority: 'high' as const, completed: false },
     { id: 2, priority: 'high' as const, completed: false },
     { id: 3, priority: 'medium' as const, completed: false },
     { id: 4, priority: 'low' as const, completed: false },
   ] as Task[]);

   // Verwerk alleen terwijl prioriteit hoog is
   tasks$.pipe(
     takeWhile(task => task.priority === 'high')
   ).subscribe(task => {
     console.log(`Verwerken taak ${task.id}`);
   });
   // Output: Verwerken taak 1, Verwerken taak 2
   ```

3. **Pagineringsverwerking**
   ```ts
   import { range } from 'rxjs';
   import { takeWhile, map } from 'rxjs';

   interface Page {
     pageNumber: number;
     hasMore: boolean;
   }

   const pages$ = range(1, 10).pipe(
     map(pageNum => ({
       pageNumber: pageNum,
       hasMore: pageNum < 5
     } as Page))
   );

   // Laad pagina's alleen terwijl hasMore waar is
   pages$.pipe(
     takeWhile(page => page.hasMore, true) // inclusive: true
   ).subscribe(page => {
     console.log(`Laden pagina ${page.pageNumber}`);
   });
   // Output: Laden pagina 1~5
   ```


## 🧠 Praktisch codevoorbeeld (Optellen limiet)

Voorbeeld van doorgaan met optellen tot een specifieke voorwaarde bereikt is.

```ts
import { fromEvent, interval } from 'rxjs';
import { takeWhile, scan, switchMap } from 'rxjs';

// Maak UI-elementen
const container = document.createElement('div');
document.body.appendChild(container);

const startButton = document.createElement('button');
startButton.textContent = 'Start Tellen';
container.appendChild(startButton);

const counter = document.createElement('div');
counter.style.fontSize = '24px';
counter.style.marginTop = '10px';
counter.textContent = 'Teller: 0';
container.appendChild(counter);

const message = document.createElement('div');
message.style.marginTop = '5px';
message.style.color = 'gray';
message.textContent = 'Telt terwijl onder 10';
container.appendChild(message);

// Start tellen bij knopklik
fromEvent(startButton, 'click').pipe(
  switchMap(() =>
    interval(500).pipe(
      scan(count => count + 1, 0),
      takeWhile(count => count < 10)
    )
  )
).subscribe({
  next: (count) => {
    counter.textContent = `Teller: ${count}`;
    startButton.disabled = true;
  },
  complete: () => {
    message.textContent = 'Voltooid na bereiken van 10!';
    message.style.color = 'green';
    startButton.disabled = false;
  }
});
```

Deze code telt op van 0 tot 9 en voltooit automatisch net voor het bereiken van 10.


## 🎯 Contrast met skipWhile

`takeWhile` en `skipWhile` hebben contrasterend gedrag.

```ts
import { range } from 'rxjs';
import { takeWhile, skipWhile } from 'rxjs';

const numbers$ = range(0, 10);

// takeWhile: Neem terwijl voorwaarde voldaan is
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4

// skipWhile: Sla over terwijl voorwaarde voldaan is
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Output: 5, 6, 7, 8, 9
```

| Operator | Gedrag | Voltooiingstiming |
|---|---|---|
| `takeWhile(predicate)` | **Neem** terwijl voorwaarde voldaan is | Wanneer voorwaarde `false` wordt |
| `skipWhile(predicate)` | **Sla over** terwijl voorwaarde voldaan is | Wanneer bronstream voltooit |


## 📋 Type-veilig gebruik

Type-veilig implementatievoorbeeld met gebruik van TypeScript generics.

```ts
import { Observable, from } from 'rxjs';
import { takeWhile } from 'rxjs';

interface SensorReading {
  timestamp: Date;
  value: number;
  unit: string;
  status: 'normal' | 'warning' | 'critical';
}

function getReadingsUntilWarning(
  readings$: Observable<SensorReading>
): Observable<SensorReading> {
  return readings$.pipe(
    takeWhile(reading => reading.status === 'normal')
  );
}

// Gebruiksvoorbeeld
const readings$ = from([
  { timestamp: new Date(), value: 25, unit: '°C', status: 'normal' as const },
  { timestamp: new Date(), value: 28, unit: '°C', status: 'normal' as const },
  { timestamp: new Date(), value: 32, unit: '°C', status: 'warning' as const },
  { timestamp: new Date(), value: 35, unit: '°C', status: 'critical' as const },
] as SensorReading[]);

getReadingsUntilWarning(readings$).subscribe(reading => {
  console.log(`${reading.value}${reading.unit} - ${reading.status}`);
});
// Output:
// 25°C - normal
// 28°C - normal
```


## 🔄 Verschil tussen takeWhile en filter

`takeWhile` verschilt van `filter` doordat het voltooit.

```ts
import { range } from 'rxjs';
import { takeWhile, filter } from 'rxjs';

const numbers$ = range(0, 10);

// filter: Laat alleen waarden door die aan voorwaarde voldoen (stream gaat door)
numbers$.pipe(
  filter(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('filter voltooid')
});
// Output: 0, 1, 2, 3, 4, filter voltooid

// takeWhile: Alleen terwijl voorwaarde voldaan is (voltooit wanneer onwaar)
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe({
  next: console.log,
  complete: () => console.log('takeWhile voltooid')
});
// Output: 0, 1, 2, 3, 4, takeWhile voltooid
```

| Operator | Gedrag | Stream voltooiing |
|---|---|---|
| `filter(predicate)` | Laat alleen waarden door die aan voorwaarde voldoen | Wanneer bronstream voltooit |
| `takeWhile(predicate)` | Neem terwijl voorwaarde voldaan is | Wanneer voorwaarde `false` wordt |


## ⚠️ Veelgemaakte fouten

> [!NOTE]
> `takeWhile` voltooit zonder iets uit te voeren als de voorwaarde `false` is vanaf het begin. Zorg ervoor dat de voorwaarde correct is ingesteld.

### Fout: Voorwaarde is onwaar vanaf het begin

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

// ❌ Slecht voorbeeld: Voorwaarde is onwaar bij eerste waarde
range(5, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Niets uitgevoerd (voorwaarde is onwaar bij eerste waarde 5)
```

### Correct: Verifieer voorwaarde

```ts
import { range } from 'rxjs';
import { takeWhile } from 'rxjs';

// ✅ Goed voorbeeld: Stel voorwaarde correct in
range(0, 10).pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4
```


## 🎓 Samenvatting

### Wanneer takeWhile gebruiken
- ✅ Wanneer u stream wilt controleren met dynamische voorwaarden
- ✅ Wanneer u data wilt acquiseren tot een drempelwaarde
- ✅ Wanneer u alleen wilt verwerken terwijl een specifieke status doorgaat
- ✅ Wanneer voorwaarde-gebaseerde vroege voltooiing nodig is

### Wanneer take gebruiken
- ✅ Wanneer het aantal op te halen vast is
- ✅ Wanneer eenvoudige aantallimiet nodig is

### Wanneer filter gebruiken
- ✅ Wanneer u alleen waarden wilt extraheren die aan een voorwaarde voldoen uit de hele stream
- ✅ Wanneer u de stream niet wilt voltooien

### Opmerkingen
- ⚠️ Als voorwaarde `false` is vanaf het begin, voltooit zonder iets uit te voeren
- ⚠️ Standaard worden waarden waarvoor voorwaarde `false` wordt niet uitgevoerd (kan worden opgenomen met `inclusive: true`)
- ⚠️ Met oneindige streams waar voorwaarde altijd `true` is, gaat het voor altijd door


## 🚀 Volgende stappen

- **[take](/nl/guide/operators/filtering/take)** - Leer hoe u eerste N waarden neemt
- **[takeLast](/nl/guide/operators/filtering/takeLast)** - Leer hoe u laatste N waarden neemt
- **[takeUntil](../utility/takeUntil)** - Leer hoe u waarden neemt tot andere Observable vuurt
- **[filter](/nl/guide/operators/filtering/filter)** - Leer hoe u filtert op basis van voorwaarden
- **[Filteroperator praktische voorbeelden](/nl/guide/operators/filtering/practical-use-cases)** - Leer echte use cases
