---
description: "L'operatore ignoreElements è un operatore di filtraggio di RxJS che ignora tutti i valori e passa solo attraverso i completamenti e gli errori. È utile quando si attende il completamento del processo."
---

# ignoreElements - passano solo i completamenti/errori

L'operatore ignoreElements **ignora tutti i valori** emessi dall'Observable di origine e solo le notifiche di **completamento e di errore** vengono passate a valle.

## 🔰 Sintassi di base e utilizzo

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valore:', value), // Non chiamato
  complete: () => console.log('Completato')
});
// Uscita: Completato
```

**Flusso operativo**:.
1. tutti i punti 1, 2, 3, 4 e 5 vengono ignorati
2. solo le notifiche di completamento vengono passate a valle

[🌐 Documentazione ufficiale di RxJS - ignoreElements](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Tipico modello di utilizzo.

- **Attendere il completamento del processo**: quando non si ha bisogno del valore e si vuole solo conoscere il completamento.
- **Eseguire solo gli effetti collaterali**: eseguire gli effetti collaterali con tap e ignorare i valori.
- **Gestione degli errori**: quando si desidera catturare solo gli errori.
- **Sincronizzazione delle sequenze**: attesa del completamento di più processi

## 🧠 Esempio pratico di codice 1: Attendere il completamento del processo di inizializzazione

Questo è un esempio di attesa del completamento di più processi di inizializzazione.

```ts
import { from, forkJoin, of } from 'rxjs';
import { ignoreElements, tap, delay, concat } from 'rxjs';

// UICreato
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Inizializzazione dell'applicazione';
container.appendChild(title);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
container.appendChild(statusArea);

const completeMessage = document.createElement('div');
completeMessage.style.marginTop = '10px';
completeMessage.style.padding = '10px';
completeMessage.style.display = 'none';
container.appendChild(completeMessage);

// Funzione per aggiungere il registro di stato
function addLog(message: string, color: string = 'black') {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
  log.style.color = color;
  statusArea.appendChild(log);
}

// Processo di inizializzazione1: Connessione al database
const initDatabase$ = from(['DBConnessione...', 'Verifica della tabella...', 'DBPronto']).pipe(
  tap(msg => addLog(msg, 'blue')),
  delay(500),
  ignoreElements() // Valori ignorati, viene notificato solo il completamento
);

// Processo di inizializzazione2: File di configurazione in lettura
const loadConfig$ = from(['File di configurazione in lettura...', 'Analisi della configurazione in corso...', 'Applicazione di configurazione completata']).pipe(
  tap(msg => addLog(msg, 'green')),
  delay(700),
  ignoreElements()
);

// Processo di inizializzazione3: Autenticazione dell'utente
const authenticate$ = from(['Informazioni di autenticazione in corso di verifica...', 'Verifica del token in corso...', 'Autenticazione completata']).pipe(
  tap(msg => addLog(msg, 'purple')),
  delay(600),
  ignoreElements()
);

// Tutti i processi di inizializzazione vengono eseguiti.
addLog('Inizializzazione avviata...', 'orange');

forkJoin([
  initDatabase$,
  loadConfig$,
  authenticate$
]).subscribe({
  complete: () => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#e8f5e9';
    completeMessage.style.color = 'green';
    completeMessage.style.fontWeight = 'bold';
    completeMessage.textContent = '✅ Tutta l'inizializzazione è stata completata.！L'applicazione può essere avviata.';
    addLog('Applicazione avviata', 'green');
  },
  error: err => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#ffebee';
    completeMessage.style.color = 'red';
    completeMessage.textContent = `❌ Errore di inizializzazione: ${err.message}`;
  }
});
```

- Viene visualizzato un registro dettagliato di ciascun processo di inizializzazione, ma i valori vengono ignorati.
- Quando tutti i processi sono stati completati, viene visualizzato un messaggio di completamento.

## 🎯 Esempio pratico di codice 2: Attesa del completamento del caricamento dei file

Questo è un esempio di visualizzazione dell'avanzamento del caricamento di più file, ma con la sola notifica del completamento.

```ts
import { from, of, concat } from 'rxjs';
import { ignoreElements, tap, delay, mergeMap } from 'rxjs';

// UICreato
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Caricamento dei file';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Caricamento avviato';
container.appendChild(button);

const progressArea = document.createElement('div');
progressArea.style.marginTop = '10px';
container.appendChild(progressArea);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.display = 'none';
container.appendChild(result);

interface FileUpload {
  name: string;
  size: number;
}

const files: FileUpload[] = [
  { name: 'document.pdf', size: 2500 },
  { name: 'image.jpg', size: 1800 },
  { name: 'video.mp4', size: 5000 }
];

// Processo di caricamento dei file (con indicazione dell'avanzamento)
function uploadFile(file: FileUpload) {
  const fileDiv = document.createElement('div');
  fileDiv.style.marginTop = '5px';
  fileDiv.style.padding = '5px';
  fileDiv.style.border = '1px solid #ccc';
  progressArea.appendChild(fileDiv);

  const progressSteps = [0, 25, 50, 75, 100];

  return from(progressSteps).pipe(
    delay(200),
    tap(progress => {
      fileDiv.textContent = `📄 ${file.name} (${file.size}KB) - ${progress}%`;
      if (progress === 100) {
        fileDiv.style.backgroundColor = '#e8f5e9';
      }
    }),
    ignoreElements() // I valori di avanzamento vengono ignorati, viene notificato solo il completamento
  );
}

button.addEventListener('click', () => {
  button.disabled = true;
  progressArea.innerHTML = '';
  result.style.display = 'none';

  // Tutti i file vengono caricati in sequenza
  from(files).pipe(
    mergeMap(file => uploadFile(file), 2) // Max.23 file in parallelo
  ).subscribe({
    complete: () => {
      result.style.display = 'block';
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
      result.innerHTML = `
        <strong>✅ Caricamento completato</strong><br>
        ${files.length}Un file è stato caricato...
      `;
      button.disabled = false;
    },
    error: err => {
      result.style.display = 'block';
      result.style.backgroundColor = '#ffebee';
      result.style.color = 'red';
      result.textContent = `❌ Errore: ${err.message}`;
      button.disabled = false;
    }
  });
});
```

- Viene visualizzato l'avanzamento di ciascun file, ma i valori di avanzamento stessi non scorrono a valle.
- Un messaggio di completamento viene visualizzato quando tutti i caricamenti sono stati completati.

## 🆚 Confronto con operatori simili

### ignoreElements vs filter(() => false) vs take(0)

```ts
import { of } from 'rxjs';
import { ignoreElements, filter, take } from 'rxjs';

const source$ = of(1, 2, 3);

// ignoreElements: Ignorare tutti i valori, il completamento viene passato attraverso
source$.pipe(
  ignoreElements()
).subscribe({
  next: v => console.log('Valore:', v),
  complete: () => console.log('ignoreElements: Completato')
});
// Uscita: ignoreElements: Completato

// filter(() => false): Filtra tutti i valori, lascia passare il completamento
source$.pipe(
  filter(() => false)
).subscribe({
  next: v => console.log('Valore:', v),
  complete: () => console.log('filter: Completato')
});
// Uscita: filter: Completato

// take(0): Completato immediatamente
source$.pipe(
  take(0)
).subscribe({
  next: v => console.log('Valore:', v),
  complete: () => console.log('take(0): Completato')
});
// Uscita: take(0): Completato
```

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valore:', value), // Non chiamato
  complete: () => console.log('Completato')
});
// Uscita: Completato
```

**Consigliato**: usare ignoreElements()` se si vuole intenzionalmente ignorare tutti i valori. L'intento del codice sarà chiaro.

## 🔄 Gestione delle notifiche di errore.

ignoreElements` ignora i valori, ma **passa le notifiche di errore**.


```ts
import { throwError, of, concat } from 'rxjs';
import { ignoreElements, delay } from 'rxjs';

const success$ = of(1, 2, 3).pipe(
  delay(100),
  ignoreElements()
);

const error$ = concat(
  of(1, 2, 3),
  throwError(() => new Error('Si verifica un errore'))
).pipe(
  ignoreElements()
);

// Caso di successo
success$.subscribe({
  next: v => console.log('Valore:', v),
  complete: () => console.log('✅ Completato'),
  error: err => console.error('❌ Errore:', err.message)
});
// Uscita: ✅ Completato

// Caso di errore
error$.subscribe({
  next: v => console.log('Valore:', v),
  complete: () => console.log('✅ Completato'),
  error: err => console.error('❌ Errore:', err.message)
});
// Uscita: ❌ Errore: Si verifica un errore
```

## ⚠️ Note.

### 1. gli effetti collaterali vengono eseguiti

ignoreElements` ignora i valori, ma gli effetti collaterali (ad esempio, tap) vengono eseguiti.

```ts
import { of } from 'rxjs';
import { ignoreElements, tap } from 'rxjs';

of(1, 2, 3).pipe(
  tap(v => console.log('Effetti collaterali:', v)),
  ignoreElements()
).subscribe({
  next: v => console.log('Valore:', v),
  complete: () => console.log('Completato')
});
// Uscita:
// Effetti collaterali: 1
// Effetti collaterali: 2
// Effetti collaterali: 3
// Completato
```

### 2. Utilizzo con Observable

Quando viene utilizzata con Infinite Observable, la sottoscrizione dura per sempre, poiché il completamento non arriva mai.

```ts
import { interval } from 'rxjs';
import { ignoreElements, take } from 'rxjs';

// ❌ Caso negativo: Non completato
interval(1000).pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Completato') // Non chiamato
});

// ✅ Buon esempio: take Completato in
interval(1000).pipe(
  take(5),
  ignoreElements()
).subscribe({
  complete: () => console.log('Completato') // 5Chiamato dopo un secondo
});
```

### 3. Tipi in TypeScript

Il valore di ritorno di `ignoreElements` è di tipo `Observable<never>`.

```ts
import { Observable, of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const numbers$: Observable<number> = of(1, 2, 3);

// ignoreElements Il risultato di Observable<never>
const result$: Observable<never> = numbers$.pipe(
  ignoreElements()
);

result$.subscribe({
  next: value => {
    // value è di tipo never quindi questo blocco non viene eseguito
    console.log(value);
  },
  complete: () => console.log('Solo il completamento')
});
```

### 4. se il completamento non è garantito

Se l'origine non viene completata, anche l'opzione ignoreElements non verrà completata.

```ts
import { NEVER } from 'rxjs';
import { ignoreElements } from 'rxjs';

// ❌ NEVERnon completa né emette un errore
NEVER.pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Completato') // Non chiamato
});
```

## 💡 Modelli di combinazione pratici

### Schema 1: sequenza di inizializzazione

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valore:', value), // Non chiamato
  complete: () => console.log('Completato')
});
// Uscita: Completato
```

### Schema 2: Processo di pulizia

{```ts
import { from, of } from 'rxjs';
import { tap, ignoreElements, mergeMap } from 'rxjs';

interface Resource {
  id: number;
  name: string;
}

const resources: Resource[] = [
  { id: 1, name: 'Database' },
  { id: 2, name: 'Cache' },
  { id: 3, name: 'Logger' }
];

from(resources).pipe(
  mergeMap(resource =>
    of(resource).pipe(
      tap(() => console.log(`🧹 ${resource.name} Pulizia in corso...`)),
      ignoreElements()
    )
  )
).subscribe({
  complete: () => console.log('✅ Tutte le risorse sono state ripulite')
});


## 📚 Operatori correlati.

- **[filter](./filter)** - filtra i valori in base a condizioni.
- **[take](./take)** - prende solo i primi N valori.
- **[skip](./skip)** - salta i primi N valori.
- **[tap](../utility/tap)** - esegue un'azione secondaria.

## Riepilogo.

L'operatore ignoreElements ignora tutti i valori e passa solo attraverso i completamenti e gli errori.

- Ideale quando è richiesta solo la notifica del completamento.
- ✅ Gli effetti collaterali (tap) sono eseguiti
- ✅ Vengono passate anche le notifiche di errore.
- ✅ Intento più chiaro di `filtro(() => false)`.
- ⚠️ L'Observable infinito non viene completato
- ⚠️ Il tipo del valore di ritorno è `Observable<never>`.
- ⚠️ Il valore è completamente ignorato, ma vengono eseguiti gli effetti collaterali
