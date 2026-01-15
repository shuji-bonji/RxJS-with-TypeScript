---
description: "O operador last recupera apenas o último valor na conclusão do stream ou o último valor que corresponde a uma condição: Essencial para extração de estado final"
titleTemplate: ':title'
---

# last - Obter Último Valor

O operador `last` recupera o **último valor** ou **último valor que satisfaz uma condição** do stream e completa o stream.


## 🔰 Sintaxe Básica e Uso

```ts
import { from } from 'rxjs';
import { last } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Obter apenas o último valor
numbers$.pipe(
  last()
).subscribe(console.log);

// Obter apenas o último valor que satisfaz a condição
numbers$.pipe(
  last(n => n < 5)
).subscribe(console.log);

// Saída:
// 5
// 4
```

- `last()` emite o **último valor emitido** quando o stream é completado.
- Se uma condição for passada, apenas o **último valor** que satisfaz a condição será recuperado.
- Se nenhum valor correspondente à condição existir, um erro é gerado.

[🌐 Documentação Oficial do RxJS - `last`](https://rxjs.dev/api/operators/last)


## 💡 Padrões de Uso Típicos

- Obter o último elemento de dados filtrados
- Recuperar o estado mais recente na conclusão do stream
- Recuperar a última operação significativa no log de sessão ou operação


## 🧠 Exemplo de Código Prático (com UI)

Recuperar e exibir o último valor que foi menor que 5 dos múltiplos valores inseridos.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, take, last } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>Exemplo Prático de last:</h3>';
document.body.appendChild(output);

// Criar campo de entrada
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Digite um número e pressione Enter';
document.body.appendChild(input);

// Stream de evento de entrada
fromEvent<KeyboardEvent>(input, 'keydown')
  .pipe(
    filter((e) => e.key === 'Enter'),
    map(() => parseInt(input.value, 10)),
    take(5), // Completar quando apenas os primeiros 5 valores são obtidos
    filter((n) => !isNaN(n) && n < 5), // Apenas passa valores menores que 5
    last() // Obter o último valor menor que 5
  )
  .subscribe({
    next: (value) => {
      const item = document.createElement('div');
      item.textContent = `Último valor menor que 5: ${value}`;
      output.appendChild(item);
    },
    complete: () => {
      const complete = document.createElement('div');
      complete.textContent = 'Completado';
      complete.style.fontWeight = 'bold';
      output.appendChild(complete);
    },
  });

```
1. Digite um número 5 vezes e pressione Enter
2. Seleciona apenas "menor que 5" dos números inseridos
3. Exibe apenas o último número inserido que é menor que 5
4. O stream completa naturalmente e termina
