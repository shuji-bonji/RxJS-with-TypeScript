---
description: "Panoramica delle Creation Functions di RxJS per eseguire comunicazioni HTTP: ajax e fromFetch. Spiegazione delle differenze, linee guida per la scelta tra le due, caratteristiche di ajax basato su XMLHttpRequest e fromFetch basato su Fetch API, gestione della cancellazione, gestione degli errori e definizione dei tipi TypeScript."
---

# Creation Functions per la comunicazione HTTP

RxJS fornisce Creation Functions per gestire le comunicazioni HTTP come Observable. In questa sezione, verranno spiegate in dettaglio le due funzioni: `ajax()` e `fromFetch()`.

## Cosa sono le Creation Functions per la comunicazione HTTP

Le Creation Functions per la comunicazione HTTP sono un gruppo di funzioni che consentono di gestire le comunicazioni con API esterne e server come stream Observable. Utilizzandole, è possibile integrare le comunicazioni HTTP asincrone nelle catene di operatori RxJS e descrivere in modo dichiarativo la gestione degli errori, i processi di retry e altro ancora.

### Caratteristiche principali

- **Comunicazioni HTTP dichiarative**: gestendo le comunicazioni HTTP come Observable, è possibile elaborare in modo dichiarativo utilizzando gli operatori
- **Gestione unificata degli errori**: unifica la gestione degli errori con operatori come `catchError()` e `retry()`
- **Cancellabile**: è possibile annullare le richieste con `unsubscribe()`
- **Integrazione con altri stream**: può essere combinato con altri Observable utilizzando `switchMap()` e simili

## Elenco delle Creation Functions per la comunicazione HTTP

| Funzione | Descrizione | Tecnologia di base | Utilizzo principale |
|------|------|-----------|---------|
| [ajax()](/it/guide/creation-functions/http-communication/ajax) | Comunicazione HTTP basata su XMLHttpRequest | XMLHttpRequest | Supporto browser legacy, monitoraggio dei progressi |
| [fromFetch()](/it/guide/creation-functions/http-communication/fromFetch) | Comunicazione HTTP basata su Fetch API | Fetch API | Browser moderni, comunicazione HTTP leggera |

## Confronto tra ajax() e fromFetch()

### Differenze fondamentali

```typescript
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { fromFetch } from 'rxjs/fetch';

// ajax() - parsing automatico della risposta
const ajax$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');
ajax$.subscribe(data => console.log(data));

// fromFetch() - parsing manuale della risposta
const fetch$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1').pipe(
  switchMap(response => response.json())
);
fetch$.subscribe(data => console.log(data));

interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}
```

### Tabella di confronto delle funzionalità

| Funzionalità | ajax() | fromFetch() |
|------|--------|-------------|
| Tecnologia di base | XMLHttpRequest | Fetch API |
| Parsing JSON automatico | ✅ Supportato con `getJSON()` | ❌ Chiamata manuale di `.json()` |
| Eventi di progresso | ✅ Supportato | ❌ Non supportato |
| Timeout | ✅ Supporto integrato | ❌ Implementazione manuale necessaria |
| Rilevamento automatico errori HTTP | ✅ Errore automatico per 4xx/5xx | ❌ Controllo manuale dello stato necessario |
| Cancellazione richiesta | ✅ Possibile con unsubscribe() | ✅ Possibile con unsubscribe() |
| Supporto IE11 | ✅ Supportato | ❌ Necessario polyfill |
| Dimensione del bundle | Leggermente grande | Piccola |

## Linee guida per la scelta

### Quando scegliere ajax()

1. **È necessario il supporto per browser legacy**
   - Quando è necessario supportare browser vecchi come IE11

2. **È necessario il monitoraggio dei progressi**
   - Quando si desidera visualizzare i progressi di upload/download dei file

3. **Recupero JSON semplice**
   - Quando si desidera recuperare facilmente JSON con `getJSON()`

4. **È necessario il rilevamento automatico degli errori**
   - Quando si desidera utilizzare il rilevamento automatico degli errori tramite codice di stato HTTP

### Quando scegliere fromFetch()

1. **Supporto solo per browser moderni**
   - Quando si supportano solo ambienti in cui è disponibile Fetch API

2. **Si desidera ridurre la dimensione del bundle**
   - Quando una funzionalità di comunicazione HTTP leggera è sufficiente

3. **Si desidera utilizzare le funzionalità di Fetch API**
   - Quando si desidera manipolare direttamente gli oggetti Request/Response
   - Quando si desidera utilizzare all'interno di Service Worker

4. **È necessario un controllo dettagliato**
   - Quando si desidera personalizzare in dettaglio l'elaborazione della risposta

## Esempi di utilizzo pratico

### Pattern di chiamata API

```typescript
import { of, catchError, retry, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

// Pattern pratico utilizzando ajax()
const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // Timeout di 5 secondi
  retry(2), // Riprova 2 volte in caso di fallimento
  catchError(error => {
    console.error('Errore recupero utente:', error);
    return of(null); // Restituisce null in caso di errore
  })
);

fetchUser$.subscribe({
  next: user => {
    if (user) {
      console.log('Utente:', user);
    } else {
      console.log('Recupero utente fallito');
    }
  }
});
```

### Pattern di invio form

```typescript
import { fromEvent, switchMap, map } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Trasforma l'evento submit del form in Observable
const form = document.querySelector('form') as HTMLFormElement;
const submit$ = fromEvent(form, 'submit').pipe(
  map(event => {
    event.preventDefault();
    const formData = new FormData(form);
    return Object.fromEntries(formData.entries());
  }),
  switchMap(data =>
    ajax.post('https://api.example.com/submit', data, {
      'Content-Type': 'application/json'
    })
  )
);

submit$.subscribe({
  next: response => console.log('Invio riuscito:', response),
  error: error => console.error('Errore invio:', error)
});
```

## Domande frequenti

### Q1: Quale dovrei usare tra ajax() e fromFetch()?

**A:** Se si supportano solo browser moderni, si consiglia `fromFetch()`. Le ragioni sono le seguenti:
- Fetch API è lo standard web più recente
- La dimensione del bundle è piccola
- Alta compatibilità futura

Tuttavia, scegli `ajax()` nei seguenti casi:
- È necessario il supporto IE11
- È necessario il monitoraggio dei progressi
- Il recupero JSON semplice è sufficiente

### Q2: Come vengono gestiti gli errori HTTP (4xx, 5xx)?

**A:**
- **ajax()**: quando il codice di stato HTTP è 400 o superiore, viene automaticamente trattato come errore e viene chiamato il callback `error`
- **fromFetch()**: anche con errori HTTP viene chiamato il callback `next`. È necessario controllare manualmente `response.ok`

### Q3: Come si annulla una richiesta?

**A:** Entrambi possono essere annullati con `unsubscribe()`.

```typescript
const subscription = ajax.getJSON('/api/data').subscribe(...);

// Annulla dopo 3 secondi
setTimeout(() => subscription.unsubscribe(), 3000);
```

## Prossimi passi

Per l'uso dettagliato di ciascuna funzione, fare riferimento alle seguenti pagine:

- [Dettagli di ajax()](/it/guide/creation-functions/http-communication/ajax) - Comunicazione HTTP basata su XMLHttpRequest
- [Dettagli di fromFetch()](/it/guide/creation-functions/http-communication/fromFetch) - Comunicazione HTTP basata su Fetch API

## Risorse di riferimento

- [RxJS Official Documentation - ajax](https://rxjs.dev/api/ajax/ajax)
- [RxJS Official Documentation - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/it/docs/Web/API/Fetch_API)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/it/docs/Web/API/XMLHttpRequest)
