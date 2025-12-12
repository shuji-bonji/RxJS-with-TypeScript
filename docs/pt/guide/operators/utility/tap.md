---
description: O operador tap é um utility operator que permite que efeitos colaterais sejam executados sem afetar o valor do stream. Ideal para depuração com saída de log, controle de estados de carregamento, rastreamento de análise, monitoramento de erros e outras aplicações onde o processamento externo é executado enquanto observa o stream. Os efeitos colaterais podem ser gerenciados em código declarativo mantendo a segurança de tipo do TypeScript.
---

# tap - Executar Efeitos Colaterais

O operador `tap` é usado para "executar efeitos colaterais sem modificar o stream."
Ideal para logging, depuração ou outras operações que não afetam valores.

## 🔰 Sintaxe Básica e Operação

Utilizado em situações onde você deseja adicionar apenas efeitos colaterais sem alterar o fluxo de valores.

```ts
import { of, tap } from 'rxjs';

of(42).pipe(
  tap(value => console.log('tap:', value))
).subscribe();
// Saída:
// tap: 42
```

Neste exemplo, o valor emitido de `of(42)` é registrado conforme passa por `tap`.
Como `tap` passa o valor "como está", não tem efeito no conteúdo do stream.

[🌐 Documentação Oficial do RxJS - tap](https://rxjs.dev/api/index/function/tap)

## 💡 Casos de Uso Típicos

`tap` é frequentemente usado para os seguintes propósitos:

- Depuração e logging
- Alternar estado de carregamento
- Exibir notificações toast
- Disparar atualizações de UI

```ts
import { of, tap, map } from 'rxjs';

of(Math.random()).pipe(
  tap(val => console.log('Valor recuperado:', val)),
  map(n => n > 0.5 ? 'Alto' : 'Baixo'),
  tap(label => console.log('Rótulo:', label))
).subscribe();
// Saída:
// Valor recuperado: 0.09909888881113504
// Rótulo: Baixo
```


## 🧪 Exemplo de Código Prático (com UI)

O seguinte é um exemplo de adição de logs ao DOM usando tap.

```ts
import { of } from 'rxjs';
import { tap, map } from 'rxjs';

// Elemento para saída de log
const logOutput = document.createElement('div');
document.body.appendChild(logOutput);

// Sequência de valores
of(1, 2, 3, 4, 5)
  .pipe(
    tap((val) => {
      console.log(`Valor original: ${val}`);

      // Adicionar log à UI
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: Valor ${val} passou`;
      logEntry.style.color = '#666';
      logOutput.appendChild(logEntry);
    }),
    map((val) => val * 10),
    tap((val) => {
      console.log(`Valor transformado: ${val}`);

      // Adicionar log à UI
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: Valor transformado ${val}`;
      logEntry.style.color = '#090';
      logOutput.appendChild(logEntry);
    })
  )
  .subscribe((val) => {
    // Exibir resultado final na UI
    const resultItem = document.createElement('div');
    resultItem.textContent = `Resultado: ${val}`;
    resultItem.style.fontWeight = 'bold';
    logOutput.appendChild(resultItem);
  });

```


## ✅ Resumo

- `tap` é um operador especializado para **inserir efeitos colaterais**
- **Saída de log e atualizações de UI** podem ser feitas sem alterar o fluxo de valores
- Pode ser combinado com `finalize` e `catchError` para controle mais prático
