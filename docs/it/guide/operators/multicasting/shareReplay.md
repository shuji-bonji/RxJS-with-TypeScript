---
description: shareReplay è un operatore multicast RxJS che bufferizza valori passati oltre al multicasting e li fornisce ai subscriber in ritardo. È ideale per situazioni dove vuoi ricordare valori passati e distribuirli a più subscriber, come caching di risposte API, condivisione di informazioni di configurazione e gestione stato. È possibile prevenire memory leak con le opzioni refCount e windowTime, e l'inferenza dei tipi TypeScript abilita l'elaborazione cache type-safe.
---

# shareReplay - Cache e Condivisione Valori Passati

L'operatore `shareReplay()` realizza il multicasting come `share()`, ma **ricorda anche un numero specificato di valori passati** e li fornisce ai subscriber che si uniscono dopo.

Questo abilita casi d'uso più avanzati come caching di risposte API e condivisione stato.

[📘 Documentazione Ufficiale RxJS - `shareReplay()`](https://rxjs.dev/api/index/function/shareReplay)

## 🔰 Utilizzo Base

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// Usando shareReplay (dimensione buffer 2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Sorgente: ${value}`)),
  shareReplay(2) // Buffer degli ultimi 2 valori
);

// Primo subscriber
console.log('Sottoscrizione Observer 1 iniziata');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Aggiungi secondo subscriber dopo 3.5 secondi
setTimeout(() => {
  console.log('Sottoscrizione Observer 2 iniziata - riceve ultimi 2 valori');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

### Risultato Esecuzione

```
Sottoscrizione Observer 1 iniziata
Sorgente: 0
Observer 1: 0
Sorgente: 1
Observer 1: 1
Sorgente: 2
Observer 1: 2
Sorgente: 3
Observer 1: 3
Sottoscrizione Observer 2 iniziata - riceve ultimi 2 valori
Observer 2: 2  // ← Valore passato bufferizzato
Observer 2: 3  // ← Valore passato bufferizzato
Sorgente: 4
Observer 1: 4
Observer 2: 4
```

**Punti Importanti**:
- I subscriber in ritardo possono ricevere immediatamente i valori passati bufferizzati
- I valori fino alla dimensione del buffer vengono ricordati (2 in questo esempio)

## 💡 Sintassi shareReplay()

```typescript
shareReplay(bufferSize?: number, windowTime?: number, scheduler?: SchedulerLike)
shareReplay(config: ShareReplayConfig)
```

### Parametri

| Parametro | Tipo | Descrizione | Default |
|-----------|---|------|----------|
| `bufferSize` | `number` | Numero di valori da bufferizzare | `Infinity` |
| `windowTime` | `number` | Periodo di validità del buffer (millisecondi) | `Infinity` |
| `scheduler` | `SchedulerLike` | Scheduler per controllo timing | - |

### Oggetto di Configurazione (RxJS 7+)

```typescript
interface ShareReplayConfig {
  bufferSize?: number;
  windowTime?: number;
  refCount?: boolean;  // Cancella sottoscrizione quando il conteggio subscriber raggiunge zero
  scheduler?: SchedulerLike;
}
```

## 📊 Differenza tra share e shareReplay

### Comportamento share()

```typescript
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Sorgente: ${value}`)),
  share()
);

source$.subscribe(value => console.log(`Observer 1: ${value}`));

setTimeout(() => {
  console.log('Sottoscrizione Observer 2 iniziata');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Risultato Esecuzione**:
```
Sorgente: 0
Observer 1: 0
Sorgente: 1
Observer 1: 1
Sottoscrizione Observer 2 iniziata
Sorgente: 2
Observer 1: 2
Observer 2: 2  // ← Non può ricevere valori passati (0, 1)
```

### Comportamento shareReplay()

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Sorgente: ${value}`)),
  shareReplay(2) // Buffer degli ultimi 2 valori
);

source$.subscribe(value => console.log(`Observer 1: ${value}`));

setTimeout(() => {
  console.log('Sottoscrizione Observer 2 iniziata');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Risultato Esecuzione**:
```
Sorgente: 0
Observer 1: 0
Sorgente: 1
Observer 1: 1
Sottoscrizione Observer 2 iniziata
Observer 2: 0  // ← Valore passato bufferizzato
Observer 2: 1  // ← Valore passato bufferizzato
Sorgente: 2
Observer 1: 2
Observer 2: 2
```

## 💼 Casi d'Uso Pratici

### 1. Caching Risposte API

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
  // Cache informazioni utente
  private userCache$ = ajax.getJSON<User>('https://jsonplaceholder.typicode.com/users/1').pipe(
    tap(() => console.log('Richiesta API eseguita')),
    shareReplay(1) // Cache permanente dell'ultimo 1 valore
  );

  getUser(): Observable<User> {
    return this.userCache$;
  }
}

const userService = new UserService();

// Primo componente
userService.getUser().subscribe(user => {
  console.log('Componente 1:', user);
});

// Altro componente dopo 2 secondi
setTimeout(() => {
  userService.getUser().subscribe(user => {
    console.log('Componente 2:', user); // ← Recuperato dalla cache, nessuna richiesta API
  });
}, 2000);
```

**Risultato Esecuzione**:
```
Richiesta API eseguita
Componente 1: { id: 1, name: "John" }
Componente 2: { id: 1, name: "John" }  // ← Nessuna richiesta API
```

### 2. Condivisione Informazioni di Configurazione

```typescript
import { of } from 'rxjs';
import { delay, shareReplay, tap } from 'rxjs';

// Ottieni impostazioni applicazione (eseguito solo una volta)
const appConfig$ = of({
  apiUrl: 'https://api.example.com',
  theme: 'dark',
  language: 'it'
}).pipe(
  delay(1000), // Simula caricamento
  tap(() => console.log('Impostazioni caricate')),
  shareReplay(1)
);

// Usa impostazioni in più servizi
appConfig$.subscribe(config => console.log('Servizio A:', config.apiUrl));
appConfig$.subscribe(config => console.log('Servizio B:', config.theme));
appConfig$.subscribe(config => console.log('Servizio C:', config.language));
```

**Risultato Esecuzione**:
```
Impostazioni caricate
Servizio A: https://api.example.com
Servizio B: dark
Servizio C: it
```

### 3. Cache a Tempo Limitato

```typescript
import { ajax } from 'rxjs/ajax';
import { shareReplay, tap } from 'rxjs';

// Cache solo per 5 secondi (usando dati TODO come esempio)
const todoData$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1').pipe(
  tap(() => console.log('Dati TODO recuperati')),
  shareReplay({
    bufferSize: 1,
    windowTime: 5000, // Valido per 5 secondi
    refCount: true    // Cancella sottoscrizione quando il conteggio subscriber raggiunge zero
  })
);

// Prima sottoscrizione
todoData$.subscribe(data => console.log('Ottieni 1:', data));

// Dopo 3 secondi (cache valida)
setTimeout(() => {
  todoData$.subscribe(data => console.log('Ottieni 2:', data)); // Dalla cache
}, 3000);

// Dopo 6 secondi (cache scaduta)
setTimeout(() => {
  todoData$.subscribe(data => console.log('Ottieni 3:', data)); // Nuova richiesta
}, 6000);
```

## ⚠️ Attenzione ai Memory Leak

`shareReplay()` mantiene i valori in un buffer, il che può causare memory leak se non gestito correttamente.

### Codice Problematico

```typescript
// ❌ Rischio di memory leak
const infiniteStream$ = interval(1000).pipe(
  shareReplay() // bufferSize non specificato = Infinity
);

// Questo stream continua ad accumulare valori per sempre
```

### Contromisure Raccomandate

```typescript
// ✅ Limita dimensione buffer
const safeStream$ = interval(1000).pipe(
  shareReplay(1) // Mantieni solo l'ultimo 1
);

// ✅ Usa refCount
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    refCount: true // Svuota buffer quando il conteggio subscriber raggiunge zero
  })
);

// ✅ Imposta limite di tempo
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    windowTime: 10000 // Scade in 10 secondi
  })
);
```

## 🎯 Scegliere la Dimensione del Buffer

| Dimensione Buffer | Caso d'Uso | Esempio |
|--------------|-----------|---|
| `1` | Solo ultimo stato necessario | Info utente corrente, impostazioni |
| `3-5` | Cronologia recente necessaria | Cronologia chat, cronologia notifiche |
| `Infinity` | Tutta la cronologia necessaria | Log, audit trail (usare con cautela) |

## 🔄 Operatori Correlati

- **[share()](/it/guide/operators/multicasting/share)** - Multicast semplice (nessun buffering)
- **[publish()](/it/guide/subjects/multicasting)** - Controllo multicast di basso livello
- **[ReplaySubject](/it/guide/subjects/types-of-subject)** - Subject che forma la base di shareReplay

## Riepilogo

L'operatore `shareReplay()`:
- Bufferizza valori passati e li fornisce ai subscriber in ritardo
- Ideale per caching risposte API
- Richiede attenzione ai memory leak
- Può essere usato in sicurezza con `refCount` e `windowTime`

Quando è necessaria la condivisione stato o il caching, `shareReplay()` è uno strumento molto potente, ma è importante impostare dimensione del buffer e impostazioni di scadenza appropriate.

## 🔗 Sezioni Correlate

- **[Errori Comuni e Soluzioni](/it/guide/anti-patterns/common-mistakes#4-uso-improprio-di-sharereplay)** - Uso corretto di shareReplay e contromisure per memory leak
- **[share()](/it/guide/operators/multicasting/share)** - Multicast semplice
- **[ReplaySubject](/it/guide/subjects/types-of-subject)** - Subject che forma la base di shareReplay
