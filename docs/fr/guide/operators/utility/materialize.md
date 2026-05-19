---
description: "materialize est un opérateur utilitaire RxJS qui convertit les notifications Observable (next, error, complete) en objets Notification. Il est idéal pour les situations où vous souhaitez manipuler la notification elle-même, comme le traitement des erreurs en tant que données, le débogage et la journalisation des notifications, l'enregistrement des méta-informations, etc. dematerialize vous permet de le renvoyer à son format d'origine et à la manipulation de notification sécurisée par type grâce à l'inférence de type TypeScript."
---

# materialize - Objectivation des notifications

L'opérateur `materialize` convertit les **notifications (next, error, complete) d'un Observable en objets Notification**. Cela permet de traiter les erreurs et les achèvements ainsi que les valeurs comme des données.

## 🔰 Syntaxe et comportement de base

Convertit un flux normal en un flux d'objets Notification.

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

of(1, 2, 3)
  .pipe(materialize())
  .subscribe(notification => {
    console.log(notification);
  });
// Sortie:
// Notification { kind: 'N', value: 1, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 2, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 3, error: undefined, hasValue: true }
// Notification { kind: 'C', value: undefined, error: undefined, hasValue: false }
```

Propriété `kind` de l'objet Notification :.
- `'N'` : next (valeur émise).
- `'E'` : erreur
- `'C'` : complete

[🌐 Documentation officielle RxJS - materialize](https://rxjs.dev/api/index/function/materialize)

## 💡 Cas d'utilisation typique.

- **Error datamining** : traiter les erreurs comme une partie du flux.
- **Débogage et journalisation** : suivi détaillé des notifications.
- Enregistrement des méta-informations** : enregistrement du moment et de la nature des notifications.
- Combinaison de flux contenant des erreurs** : traiter les erreurs dans plusieurs flux de manière unifiée.

## 🧪 Exemple de code pratique 1 : traiter les erreurs comme des données

Cet exemple permet de traiter les erreurs qui devraient normalement interrompre un flux comme des données et de continuer.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, map } from 'rxjs';

// UICréation
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'materialize - Analyse des données d'erreurs';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.appendChild(logItem);
}

// Traitement normal des erreurs (interruption du flux)
addLog('--- Traitement normal des erreurs ---', '#e3f2fd');
concat(
  of(1, 2),
  throwError(() => new Error('Survenance d'une erreur')),
  of(3, 4)  // Non exécuté ici
).subscribe({
  next: v => addLog(`Valeur: ${v}`, '#c8e6c9'),
  error: err => addLog(`❌ Erreur: ${err.message}`, '#ffcdd2'),
  complete: () => addLog('Terminé', '#e3f2fd')
});

// materializeUtilisation (suite du flux)
setTimeout(() => {
  addLog('--- materializeUtilisation ---', '#e3f2fd');

  concat(
    of(1, 2),
    throwError(() => new Error('Survenance d'une erreur')),
    of(3, 4)
  )
    .pipe(
      materialize(),
      map(notification => {
        if (notification.kind === 'N') {
          return `Valeur: ${notification.value}`;
        } else if (notification.kind === 'E') {
          return `Erreur (données analysées): ${notification.error?.message}`;
        } else {
          return 'Terminé';
        }
      })
    )
    .subscribe({
      next: msg => {
        const color = msg.includes('Erreur') ? '#fff9c4' : '#c8e6c9';
        addLog(msg, color);
      },
      complete: () => addLog('Flux terminé', '#e3f2fd')
    });
}, 1000);
```

- Les erreurs normales interrompent le flux.
- Avec `materialize`, les erreurs sont traitées comme des données et le flux continue

## 🧪 Exemple de code pratique 2 : journalisation pour le débogage

Exemple de sortie de journalisation détaillée de toutes les notifications.

```ts
import { interval, throwError } from 'rxjs';
import { materialize, take, mergeMap } from 'rxjs';

// UICréation
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'materialize - Enregistrement de débogage';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '250px';
output2.style.overflow = 'auto';
output2.style.fontFamily = 'monospace';
output2.style.fontSize = '12px';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('ja-JP', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.marginBottom = '2px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

interval(500)
  .pipe(
    take(5),
    mergeMap(value => {
      // Si la valeur est3Lève une erreur lorsque l'élément
      if (value === 3) {
        return throwError(() => new Error('Valeur3Erreur dans'));
      }
      return of(value);
    }),
    materialize()
  )
  .subscribe({
    next: notification => {
      switch (notification.kind) {
        case 'N':
          addLog2(`[NEXT] value: ${notification.value}`);
          break;
        case 'E':
          addLog2(`[ERROR] ${notification.error?.message}`);
          break;
        case 'C':
          addLog2('[COMPLETE]');
          break;
      }
    },
    complete: () => {
      addLog2('--- ObserverTerminé ---');
    }
  });
```

- Journalisation uniforme de tous les types de notification (next, error, complete)
- Suivi de l'ordre dans lequel les notifications se produisent avec des horodatages
- Utile pour le débogage et la surveillance

## 🆚 Comparaison avec des flux normaux

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

// Flux normal
of(1, 2, 3).subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('Terminé')
});
// Sortie:
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Terminé

// materializeUtilisation
of(1, 2, 3)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Notification:', n),
    complete: () => console.log('Terminé')
  });
// Sortie:
// Notification: Notification { kind: 'N', value: 1, ... }
// Notification: Notification { kind: 'N', value: 2, ... }
// Notification: Notification { kind: 'N', value: 3, ... }
// Notification: Notification { kind: 'C', ... }
// Terminé
```

## Manipulation d'objets de notification

```ts
import { of } from 'rxjs';
import { materialize, map } from 'rxjs';

of(10, 20, 30)
  .pipe(
    materialize(),
    map(notification => {
      // NotificationPropriétés de l'objet
      return {
        kind: notification.kind,           // 'N', 'E', 'C'
        hasValue: notification.hasValue,   // A une valeur ou
        value: notification.value,         // Valeur (sinext(si l'objet a une valeur)
        error: notification.error          // Erreur (error(si l'objet a une valeur)
      };
    })
  )
  .subscribe(console.log);
// Sortie:
// { kind: 'N', hasValue: true, value: 10, error: undefined }
// { kind: 'N', hasValue: true, value: 20, error: undefined }
// { kind: 'N', hasValue: true, value: 30, error: undefined }
// { kind: 'C', hasValue: false, value: undefined, error: undefined }
```

## ⚠️ Notes.

### 1. les erreurs n'interrompent pas le flux

Lorsque l'on utilise `materialize`, les erreurs sont traitées comme des données et le flux n'est pas interrompu.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize } from 'rxjs';

concat(
  of(1),
  throwError(() => new Error('Erreur')),
  of(2)
)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Notification:', n.kind),
    error: () => console.log('Gestionnaire d'erreur'),  // N'est pas appelé
    complete: () => console.log('Terminé')
  });
// Sortie:
// Notification: N
// Notification: E  ← Les erreurs sont égalementnexttraitées comme
// Terminé
```

### 2. en combinaison avec dematerialize

Les flux transformés avec `materialize` peuvent être restaurés avec `dematerialize`.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),
    // Un peu de traitement ici
    dematerialize()  // Annuler
  )
  .subscribe(console.log);
// Sortie: 1, 2, 3
```

### 3. impact sur les performances

La génération d'objets de notification entraîne des frais généraux. A n'utiliser qu'en cas de nécessité dans un environnement de production.

## 📚 Opérateurs associés.

- **[dematerialize](. /dematerialize)** - ramène l'objet Notification à une notification normale.
- **[tap](. /tap)** - Effectue un effet de bord (à des fins de débogage).
- **[catchError](. /... /error-handling/retry-catch)** - Gestion des erreurs.

## ✅ Résumé

L'opérateur `materialize` convertit une notification en un objet Notification.

- ✅ Peut gérer les erreurs comme des données.
- ✅ Utile pour le débogage et la journalisation.
- ✅ Peut enregistrer des méta-informations sur les notifications.
- ✅ Peut être annulé avec `dematerialize`.
- ⚠️ Les erreurs n'interrompent plus le flux
- ⚠️ Noter la surcharge de performance
