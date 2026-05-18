---
description: "dematerialize est un opérateur utilitaire RxJS qui restaure les objets Notification en notifications normales (next, error, complete) et effectue la transformation inverse de materialize. Il est idéal pour restaurer les notifications après traitement, filtrer et convertir les erreurs, réordonner et mettre en mémoire tampon les notifications, et dans d'autres situations où les notifications sont traitées comme des données puis ramenées à leur format d'origine."
---

# dematerialize - restaurer les objets de la notification

L'opérateur `dematerialize` **convertit un objet Notification en une notification normale (next, error, complete)**. Il effectue la transformation inverse de `materialize` et restaure la notification datatisée dans sa forme originale.

## 🔰 Syntaxe et comportement de base

Convertit un flux d'objets Notification en un flux normal.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),     // NotificationConvertir en objet
    dematerialize()    // Retour à l'original
  )
  .subscribe({
    next: v => console.log('Valeur:', v),
    complete: () => console.log('Terminé')
  });
// Sortie:
// Valeur: 1
// Valeur: 2
// Valeur: 3
// Terminé
```

[🌐 Documentation officielle de RxJS - dematerialize](https://rxjs.dev/api/index/function/dematerialize)

## 💡 Cas d'utilisation typique.

- **Restaurer les notifications après traitement** : après traitement avec materialize, les restaurer dans leur format d'origine.
- **Filtrer les erreurs** : exclure seulement certaines erreurs.
- Rétablir l'ordre des notifications** : restaurer après avoir trié les notifications comme des données.
- Restaurer après débogage** : revenir au fonctionnement normal après traitement, par exemple la sortie du journal.

## 🧪 Exemple de code pratique 1 : filtrage sélectif des erreurs

Il s'agit d'un exemple dans lequel seules certaines erreurs sont exclues et les autres sont traitées normalement.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize, filter } from 'rxjs';

// UICréer
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'dematerialize - Filtrage des erreurs';
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

// Flux contenant des erreurs
const source$ = concat(
  of(1, 2),
  throwError(() => new Error('Erreurs à ignorer')),
  of(3, 4),
  throwError(() => new Error('Erreurs graves')),
  of(5)
);

source$
  .pipe(
    materialize(),
    filter(notification => {
      // "Erreurs à ignorer"Filtrage uniquement
      if (notification.kind === 'E') {
        const errorMessage = notification.error?.message || '';
        if (errorMessage.includes('Doivent être ignorées')) {
          addLog(`🔇 Ignorer: ${errorMessage}`, '#fff9c4');
          return false;  // Exclure cette erreur
        }
      }
      return true;
    }),
    dematerialize()  // Revenir au format d'origine
  )
  .subscribe({
    next: v => addLog(`✅ Valeur: ${v}`, '#c8e6c9'),
    error: err => addLog(`❌ Erreur: ${err.message}`, '#ffcdd2'),
    complete: () => addLog('Terminé', '#e3f2fd')
  });
```

- Les "erreurs ignorées" sont exclues et le flux continue.
- Les "erreurs critiques" sont transmises au gestionnaire d'erreurs comme d'habitude.
- Traitement sélectif des erreurs possible

## 🧪 Exemple de code pratique 2 : Notification différée

Il s'agit d'un exemple de mise en mémoire tampon temporaire d'une notification avant de la restaurer.

```ts
import { from, interval, take, delay } from 'rxjs';
import { materialize, dematerialize, bufferTime, concatMap } from 'rxjs';

// UICréer
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'dematerialize - Mise en mémoire tampon et délai';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('ja-JP', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('Commence - 1Valeur émise toutes les secondes,2Traitement collectif toutes les secondes');

interval(1000)
  .pipe(
    take(6),
    materialize(),
    bufferTime(2000),      // 2Mise en mémoire tampon toutes les secondes
    concatMap(notifications => {
      addLog2(`--- ${notifications.length}Traitement d'un avis à partir de la mémoire tampon ---`);
      return from(notifications).pipe(
        delay(500),        // Traitement de chaque avis avec un délai de0.5seconde
        dematerialize()    // Revenir au format d'origine
      );
    })
  )
  .subscribe({
    next: v => addLog2(`Valeur: ${v}`),
    complete: () => addLog2('Terminé')
  });
```

- Mise en mémoire tampon des notifications toutes les 2 secondes.
- Récupération dans la mémoire tampon et traitement différé
- Restauré en tant que flux original avec `dematerialize`.

## 🆚 Relation avec materialize

```ts
import { of } from 'rxjs';
import { materialize, dematerialize, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),           // NotificationConverti en
    map(notification => {
      // NotificationTraité comme un objet
      console.log('kind:', notification.kind);
      return notification;
    }),
    dematerialize()          // Retour à l'original
  )
  .subscribe(v => console.log('Valeur:', v));
// Sortie:
// kind: N
// Valeur: 1
// kind: N
// Valeur: 2
// kind: N
// Valeur: 3
// kind: C
```

| Flux de processus | Description. |
|---|---|
| Flux original | Valeur normale (next), erreur (error), complete (complete) |
| ↓ `materialize()` | Flux de l'objet Notification |
| Traitement intermédiaire | Traitement et filtrage en tant que notification |
| ↓ `dematerialize()` | Rétablissement du flux normal |
| Flux final | Valeurs normales, erreurs, achèvement |

## ⚠️ Notes.

### 1. les notifications d'erreurs sont converties en erreurs réelles

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Traite chaqueObservableest converti en objet de notificationmaterialize()Converti en objet de notification avec
concat(
  of(1).pipe(materialize()),
  throwError(() => new Error('Erreur')).pipe(materialize()),
  of(2).pipe(materialize())  // Non exécuté parce qu'il fait suite à une erreur
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Valeur:', v),
    error: err => console.log('Erreur:', err.message)
  });
// Sortie:
// Valeur: 1
// Erreur: Erreur
```

Lorsqu'une notification d'erreur est atteinte, le flux est interrompu par une erreur.

### La notification d'achèvement termine le flux.

```ts
import { of, EMPTY, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Traite chaqueObservableest converti en objet de notificationmaterialize()Converti en objet de notification avec
concat(
  of(1).pipe(materialize()),
  of(2).pipe(materialize()),
  EMPTY.pipe(materialize()),  // Avis d'achèvement
  of(3).pipe(materialize())   // Non exécuté parce qu'il est après l'achèvement
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Valeur:', v),
    complete: () => console.log('Terminé')
  });
// Sortie:
// Valeur: 1
// Valeur: 2
// Terminé
```

Aucune valeur n'est émise après la notification d'achèvement.

### Objet de notification illégal.

Le `dematerialize` attend un objet Notification correct.

```ts
import { of } from 'rxjs';
import { dematerialize } from 'rxjs';

// ❌ Si une valeur normale est transmise àdematerializeErreur si une valeur normale est transmise à
of(1, 2, 3)
  .pipe(
    dematerialize()  // NotificationPas un objet
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Erreur:', err.message)
  });
// L'erreur se produit si un
```

## Exemples de combinaisons pratiques

```ts
import { interval, throwError, of, concat } from 'rxjs';
import { materialize, dematerialize, take, mergeMap, map } from 'rxjs';

// Exemple de conversion d'une erreur enwarningExemple de conversion en
interval(500)
  .pipe(
    take(10),
    mergeMap(value => {
      // 5L'erreur ne se produit que si
      if (value === 5) {
        return throwError(() => new Error(`Valeur${value}Erreur en`));
      }
      return of(value);
    }),
    materialize(),
    map(notification => {
      // Exemple de conversion d'une erreur enwarningConvertie en message
      if (notification.kind === 'E') {
        console.warn('Warning:', notification.error?.message);
        // Des valeurs spéciales sont émises à la place des erreurs (générées parmaterialize()Générées par)
        return { kind: 'N' as const, value: -1 };
      }
      return notification;
    }),
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Valeur:', v),
    error: err => console.error('Erreur:', err),  // N'est pas appelée
    complete: () => console.log('Terminé')
  });
// Sortie:
// Valeur: 0, 1, 2, 3, 4
// Warning: Valeur5Erreur en
// Valeur: -1  (Au lieu d'une erreur)
// Valeur: 6, 7, 8, 9
// Terminé
```

## 📚 Opérateurs apparentés

- **[materialize](. /materialize)** - Convertit une notification en un objet Notification.
- **[catchError](. /... /error-handling/retry-catch)** - Gestion des erreurs.
- **[retry](. /retry)** - Réessai en cas d'erreur.

## ✅ Résumé.

L'opérateur `dematerialize` ramène l'objet Notification à une notification normale.

- ✅ Conversion inverse de `materialize`.
- ✅ Restaure la notification dans son format d'origine après traitement.
- ✅ Permet le filtrage et la conversion des erreurs.
- ✅ Peut être utilisé pour changer l'ordre des notifications et les mettre en mémoire tampon.
- ⚠️ Les notifications d'erreur se comportent comme des erreurs réelles
- ⚠️ La notification d'achèvement termine le flux
- ⚠️ Nécessite un objet de notification correct
