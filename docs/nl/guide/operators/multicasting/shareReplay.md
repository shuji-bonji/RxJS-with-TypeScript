---
description: shareReplay is een RxJS multicast operator die naast multicasting ook vorige waarden buffert en aan vertraagde subscribers levert. Optimaal voor scenario's waar u vorige waarden wilt onthouden en aan meerdere subscribers wilt distribueren, zoals API response caching, configuratie-informatie delen en statusbeheer. Voorkomen van geheugenlekkages is ook mogelijk met refCount en windowTime opties, en realiseer type-veilig cache verwerking met TypeScript type-inferentie.
---

# shareReplay - Cache en delen

De `shareReplay()` operator realiseert multicasting net als `share()`, maar **onthoudt daarnaast een opgegeven aantal vorige waarden** en levert deze ook aan later deelnemende subscribers.

Hierdoor kunt u geavanceerdere use cases ondersteunen, zoals het cachen van API-responsen en het delen van status.

[📘 RxJS Officiële Documentatie - `shareReplay()`](https://rxjs.dev/api/index/function/shareReplay)

## 🔰 Basis Gebruik

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// Gebruik shareReplay (buffergrootte 2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Bron: ${value}`)),
  shareReplay(2) // Buffer laatste 2 waarden
);

// Eerste subscriber
console.log('Observer 1 subscriptie starten');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 3,5 seconden later tweede subscriber toevoegen
setTimeout(() => {
  console.log('Observer 2 subscriptie starten - Ontvang laatste 2 waarden');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

### Uitvoeringsresultaat

```
Observer 1 subscriptie starten
Bron: 0
Observer 1: 0
Bron: 1
Observer 1: 1
Bron: 2
Observer 1: 2
Bron: 3
Observer 1: 3
Observer 2 subscriptie starten - Ontvang laatste 2 waarden
Observer 2: 2  // ← Gebufferde vorige waarde
Observer 2: 3  // ← Gebufferde vorige waarde
Bron: 4
Observer 1: 4
Observer 2: 4
```

**Belangrijke Punten**:
- Vertraagde subscribers kunnen ook direct gebufferde vorige waarden ontvangen
- Waarden ter grootte van buffergrootte worden onthouden (in dit voorbeeld 2)

## 💡 Syntax van shareReplay()

```typescript
shareReplay(bufferSize?: number, windowTime?: number, scheduler?: SchedulerLike)
shareReplay(config: ShareReplayConfig)
```

### Parameters

| Parameter | Type | Beschrijving | Standaard |
|-----------|------|--------------|-----------|
| `bufferSize` | `number` | Aantal te bufferen waarden | `Infinity` |
| `windowTime` | `number` | Geldigheidsduur van buffer (milliseconden) | `Infinity` |
| `scheduler` | `SchedulerLike` | Scheduler voor timing controle | - |

### Configuratie Object (RxJS 7+)

```typescript
interface ShareReplayConfig {
  bufferSize?: number;
  windowTime?: number;
  refCount?: boolean;  // Of te annuleren wanneer subscribers 0 worden
  scheduler?: SchedulerLike;
}
```

## 📊 Verschil tussen share en shareReplay

### Werking van share()

```typescript
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Bron: ${value}`)),
  share()
);

source$.subscribe(value => console.log(`Observer 1: ${value}`));

setTimeout(() => {
  console.log('Observer 2 subscriptie starten');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Uitvoeringsresultaat**:
```
Bron: 0
Observer 1: 0
Bron: 1
Observer 1: 1
Observer 2 subscriptie starten
Bron: 2
Observer 1: 2
Observer 2: 2  // ← Kan vorige waarden (0, 1) niet ontvangen
```

### Werking van shareReplay()

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Bron: ${value}`)),
  shareReplay(2) // Buffer laatste 2 waarden
);

source$.subscribe(value => console.log(`Observer 1: ${value}`));

setTimeout(() => {
  console.log('Observer 2 subscriptie starten');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Uitvoeringsresultaat**:
```
Bron: 0
Observer 1: 0
Bron: 1
Observer 1: 1
Observer 2 subscriptie starten
Observer 2: 0  // ← Gebufferde vorige waarde
Observer 2: 1  // ← Gebufferde vorige waarde
Bron: 2
Observer 1: 2
Observer 2: 2
```

## 💼 Praktische Use Cases

### 1. Cachen van API Responsen

```typescript
import { Observable } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, shareReplay, tap } from 'rxjs';

interface User {
  id: number;
  name: string;
  username: string;
  email: string;
}

class UserService {
  // Cache gebruikersinformatie
  private userCache$ = ajax.getJSON<User>('https://jsonplaceholder.typicode.com/users/1').pipe(
    tap(() => console.log('API verzoek uitvoeren')),
    shareReplay(1) // Cache laatste 1 waarde permanent
  );

  getUser(): Observable<User> {
    return this.userCache$;
  }
}

const userService = new UserService();

// Eerste component
userService.getUser().subscribe(user => {
  console.log('Component 1:', user);
});

// 2 seconden later ander component
setTimeout(() => {
  userService.getUser().subscribe(user => {
    console.log('Component 2:', user); // ← Ophalen uit cache, geen API verzoek
  });
}, 2000);
```

**Uitvoeringsresultaat**:
```
API verzoek uitvoeren
Component 1: { id: 1, name: "John" }
Component 2: { id: 1, name: "John" }  // ← Geen API verzoek
```

### 2. Configuratie-informatie Delen

```typescript
import { of } from 'rxjs';
import { delay, shareReplay, tap } from 'rxjs';

// Applicatie configuratie ophalen (alleen eerste keer uitvoeren)
const appConfig$ = of({
  apiUrl: 'https://api.example.com',
  theme: 'dark',
  language: 'nl'
}).pipe(
  delay(1000), // Simuleer laden
  tap(() => console.log('Configuratie geladen')),
  shareReplay(1)
);

// Configuratie gebruiken in meerdere services
appConfig$.subscribe(config => console.log('Service A:', config.apiUrl));
appConfig$.subscribe(config => console.log('Service B:', config.theme));
appConfig$.subscribe(config => console.log('Service C:', config.language));
```

**Uitvoeringsresultaat**:
```
Configuratie geladen
Service A: https://api.example.com
Service B: dark
Service C: nl
```

### 3. Cache met Tijdslimiet

```typescript
import { ajax } from 'rxjs/ajax';
import { shareReplay, tap } from 'rxjs';

// Cache slechts 5 seconden (gebruik TODO data als voorbeeld)
const todoData$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1').pipe(
  tap(() => console.log('TODO data ophalen')),
  shareReplay({
    bufferSize: 1,
    windowTime: 5000, // 5 seconden geldig
    refCount: true    // Annuleren wanneer subscribers 0 worden
  })
);

// Eerste subscriptie
todoData$.subscribe(data => console.log('Ophalen 1:', data));

// 3 seconden later (cache geldig)
setTimeout(() => {
  todoData$.subscribe(data => console.log('Ophalen 2:', data)); // Van cache
}, 3000);

// 6 seconden later (cache verlopen)
setTimeout(() => {
  todoData$.subscribe(data => console.log('Ophalen 3:', data)); // Nieuw verzoek
}, 6000);
```

## ⚠️ Let op Geheugenlekkages

`shareReplay()` houdt waarden vast in de buffer, dus zonder goed beheer kan het leiden tot geheugenlekkages.

### Problematische Code

```typescript
// ❌ Risico op geheugenlekkage
const infiniteStream$ = interval(1000).pipe(
  shareReplay() // bufferSize niet opgegeven = Infinity
);

// Deze stream blijft voor altijd waarden accumuleren
```

### Aanbevolen Maatregelen

```typescript
// ✅ Beperk buffergrootte
const safeStream$ = interval(1000).pipe(
  shareReplay(1) // Behoud alleen laatste 1
);

// ✅ Gebruik refCount
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    refCount: true // Wis buffer wanneer subscribers 0 worden
  })
);

// ✅ Stel tijdslimiet in
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    windowTime: 10000 // Verloopt na 10 seconden
  })
);
```

## 🎯 Buffergrootte Kiezen

| Buffergrootte | Use Case | Voorbeeld |
|--------------|----------|-----------|
| `1` | Alleen laatste status nodig | Huidige gebruikersinformatie, configuratie |
| `3-5` | Recente geschiedenis nodig | Chat geschiedenis, notificatie geschiedenis |
| `Infinity` | Alle geschiedenis nodig | Logs, audit trail (voorzichtig) |

## 🔄 Gerelateerde Operators

- **[share()](/nl/guide/operators/multicasting/share)** - Eenvoudige multicast (zonder buffer)
- **[publish()](/nl/guide/subjects/multicasting)** - Low-level multicast controle
- **[ReplaySubject](/nl/guide/subjects/types-of-subject)** - Subject die de basis vormt voor shareReplay

## Samenvatting

De `shareReplay()` operator,
- Buffert vorige waarden en levert deze ook aan vertraagde subscribers
- Optimaal voor cachen van API responsen
- Let op geheugenlekkages
- Veilig te gebruiken met `refCount` en `windowTime`

Wanneer status delen of caching nodig is, is `shareReplay()` een zeer krachtig hulpmiddel, maar het is belangrijk om de juiste buffergrootte en verloopdatum in te stellen.

## 🔗 Gerelateerde Secties

- **[Veelvoorkomende Fouten en Oplossingen](/nl/guide/anti-patterns/common-mistakes#4-sharereplay-の誤用)** - Juist gebruik van shareReplay en maatregelen tegen geheugenlekkages
- **[share()](/nl/guide/operators/multicasting/share)** - Eenvoudige multicast
- **[ReplaySubject](/nl/guide/subjects/types-of-subject)** - Subject die de basis vormt voor shareReplay
